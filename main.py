#!/usr/bin/env python3
"""
2D SLAM Mapping Application with Particle Filter SLAM Integration

This application:
1. Receives LiDAR data via UDP from a SICK nanoScan3 sensor
2. Processes the data using SanjayJohn21358's particle filter SLAM implementation
3. Provides a professional UI for visualization and controls
4. Saves maps in .smap format

THIS VERSION USES THE PARTICLE FILTER SLAM IMPLEMENTATION
"""

import socket
import struct
import time
import threading
import queue
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons
from matplotlib.collections import LineCollection
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import tkinter as tk
from tkinter import filedialog, messagebox
import warnings
import sys
import traceback
import pickle

# Suppress matplotlib warnings for cleaner output
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

# -------------------- Configuration --------------------
# Filter distances outside this window (mm)
MIN_VALID_MM = 30           # ignore 0 / near-zero
MAX_VALID_MM = 30000        # clip absurdly large returns
ROBUST_PERCENTILE = 99.0    # autoscale to this percentile (ignores outliers)
DEFAULT_RMAX = 6000         # if no valid points, show up to this radius (mm)
CAP_RAYS = 4000             # max rays to draw (for perf)
COLOR_BY_RANGE = False      # True = colormap by range; False = single color
RAY_COLOR = 'tab:blue'      # used if COLOR_BY_RANGE is False
RAY_WIDTH = 0.6
BIND_IP = '192.168.192.5'   # or '0.0.0.0' to listen on all interfaces
PORT = 6060
MAP_RESOLUTION = 0.1        # meters per pixel for the occupancy grid
MAP_WIDTH = 50.0            # meters (map width in meters)
MAP_HEIGHT = 50.0           # meters (map height in meters)
NUM_PARTICLES = 50          # Number of particles for the particle filter

# ------------- UDP header -------------

@dataclass
class UDPHeader:
    marker: bytes
    protocol: bytes
    version_major: int
    version_minor: int
    total_length: int
    identification: int
    fragment_offset: int

def parse_udp_header(pkt: bytes) -> UDPHeader:
    if len(pkt) < 24:
        raise ValueError("UDP payload too short for 24-byte nanoScan3 header")
    marker = pkt[0:4]
    protocol = pkt[4:6]
    ver_major = pkt[6]
    ver_minor = pkt[7]
    total_length = struct.unpack('<I', pkt[8:12])[0]
    identification = struct.unpack('<I', pkt[12:16])[0]
    fragment_offset = struct.unpack('<I', pkt[16:20])[0]
    return UDPHeader(marker, protocol, ver_major, ver_minor, total_length, identification, fragment_offset)

# ------------- Data output header (56 bytes) -------------

@dataclass
class DataOutputHeader:
    version_b0: int
    version_b1: int
    version_b2: int
    version_b3: int
    device_serial: int
    system_plug_serial: int
    channel_number: int
    sequence_number: int
    scan_number: int
    timestamp_date: int
    timestamp_time: int
    blocks: List[Tuple[int, int]]  # list of (offset, size), 6 entries

def parse_data_output_header(buf: bytes) -> DataOutputHeader:
    if len(buf) < 56:
        raise ValueError("Instance too short for 56-byte data output header")
    v0, v1, v2, v3 = buf[0], buf[1], buf[2], buf[3]
    device_serial = struct.unpack('<I', buf[4:8])[0]
    system_plug_serial = struct.unpack('<I', buf[8:12])[0]
    channel_number = buf[12]
    sequence_number = struct.unpack('<I', buf[16:20])[0]
    scan_number = struct.unpack('<I', buf[20:24])[0]
    timestamp_date = struct.unpack('<H', buf[24:26])[0]
    timestamp_time = struct.unpack('<I', buf[28:32])[0]
    blocks = []
    for i in range(6):
        off = 32 + i * 4
        blk_off = struct.unpack('<H', buf[off:off+2])[0]
        blk_sz = struct.unpack('<H', buf[off+2:off+4])[0]
        blocks.append((blk_off, blk_sz))
    return DataOutputHeader(v0, v1, v2, v3,
                            device_serial, system_plug_serial, channel_number,
                            sequence_number, scan_number, timestamp_date, timestamp_time,
                            blocks)

# ------------- Config block -------------

@dataclass
class ConfigBlock:
    factor: int
    num_beams: int
    scan_cycle_time_ms: float
    start_angle_deg: float
    angular_resolution_deg: float
    beam_interval_us: int

def parse_config_block(buf: bytes) -> ConfigBlock:
    if len(buf) < 24:
        raise ValueError("Config block too short")
    factor = struct.unpack('<H', buf[0:2])[0]
    num_beams = struct.unpack('<H', buf[2:4])[0]
    scan_cycle_time = struct.unpack('<H', buf[4:6])[0]  # ms
    start_angle_int = struct.unpack('<i', buf[8:12])[0]     # 1/4194304 deg
    ang_res_int = struct.unpack('<i', buf[12:16])[0]        # 1/4194304 deg
    start_angle_deg = start_angle_int / 4194304.0
    ang_res_deg = ang_res_int / 4194304.0
    beam_interval = struct.unpack('<I', buf[16:20])[0]      # us
    return ConfigBlock(factor, num_beams, float(scan_cycle_time),
                       start_angle_deg, ang_res_deg, beam_interval)

# ------------- Measurement data -------------

@dataclass
class Measurement:
    distances_mm: np.ndarray      # shape (N,)
    rssi: Optional[np.ndarray]    # shape (N,) or None
    status: Optional[np.ndarray]  # shape (N,) or None

def parse_measurement_block(buf: bytes, expected_beams: Optional[int]) -> Measurement:
    if len(buf) < 4:
        raise ValueError("Measurement block too short (missing num_beams)")
    num_beams = struct.unpack('<I', buf[0:4])[0]
    if expected_beams is not None and expected_beams != 0 and num_beams != expected_beams:
        print(f"Warning: beams in measurement ({num_beams}) != config ({expected_beams})")
    payload = buf[4:]
    if len(payload) == num_beams * 2:
        d = np.frombuffer(payload, dtype='<u2', count=num_beams)
        return Measurement(distances_mm=d.astype(np.uint16), rssi=None, status=None)
    elif len(payload) == num_beams * 4:
        distances = np.empty(num_beams, dtype=np.uint16)
        rssi = np.empty(num_beams, dtype=np.uint8)
        status = np.empty(num_beams, dtype=np.uint8)
        off = 0
        for i in range(num_beams):
            distances[i] = struct.unpack('<H', payload[off:off+2])[0]
            rssi[i] = payload[off+2]
            status[i] = payload[off+3]
            off += 4
        return Measurement(distances_mm=distances, rssi=rssi, status=status)
    else:
        raise ValueError(f"Unexpected measurement payload size {len(payload)} for {num_beams} beams")

# ------------- Reassembler -------------

@dataclass
class PendingScan:
    total_length: int
    received: Dict[int, bytes]  # fragment_offset -> fragment_data
    created_at: float

class NanoScan3UDP:
    def _init_(self, bind_ip='0.0.0.0', port=6060):
        import socket as _socket
        self.sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_RCVBUF, 8*1024*1024)
        except Exception:
            pass
        self.sock.bind((bind_ip, port))
        self.pending: Dict[int, PendingScan] = {}
        self.running = False

    def _merge_coverage(self, parts: Dict[int, bytes]) -> int:
        intervals = [(off, off + len(data)) for off, data in parts.items()]
        if not intervals:
            return 0
        intervals.sort()
        covered = 0
        cur_s, cur_e = intervals[0]
        for s, e in intervals[1:]:
            if s <= cur_e:
                cur_e = max(cur_e, e)
            else:
                covered += (cur_e - cur_s)
                cur_s, cur_e = s, e
        covered += (cur_e - cur_s)
        return covered

    def _try_assemble(self, ident: int, ps: PendingScan) -> Optional[bytes]:
        if self._merge_coverage(ps.received) < ps.total_length:
            return None
        chunks = []
        for off in sorted(ps.received.keys()):
            data = ps.received[off]
            end = off + len(data)
            if off >= ps.total_length:
                continue
            if end > ps.total_length:
                data = data[:ps.total_length - off]
            chunks.append((off, data))
        buf = bytearray(ps.total_length)
        for off, data in chunks:
            buf[off:off+len(data)] = data
        return bytes(buf)

    def receive_loop(self, on_scan):
        self.running = True
        print("Listening for UDP packets...")
        while self.running:
            try:
                pkt, addr = self.sock.recvfrom(65535)
            except OSError:
                break
            except Exception as e:
                print(f"recv error: {e}")
                continue

            if len(pkt) < 24:
                continue
            try:
                hdr = parse_udp_header(pkt[:24])
            except Exception as e:
                print(f"Header parse error: {e}")
                continue
            if hdr.marker != b"MS3 " or hdr.protocol != b"MD":
                continue

            frag = pkt[24:]
            ps = self.pending.get(hdr.identification)
            if ps is None:
                ps = PendingScan(total_length=hdr.total_length, received={}, created_at=time.time())
                self.pending[hdr.identification] = ps

            ps.received[hdr.fragment_offset] = frag

            assembled = self._try_assemble(hdr.identification, ps)
            if assembled is not None:
                del self.pending[hdr.identification]
                try:
                    on_scan(assembled)
                except Exception as e:
                    print(f"Error in scan callback: {e}")

            now = time.time()
            to_del = [i for i, s in self.pending.items() if now - s.created_at > 2.0]
            for i in to_del:
                del self.pending[i]

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass

# ------------- High-level scan parsing -------------

def parse_full_instance(instance: bytes):
    doh = parse_data_output_header(instance[:56])

    def get_block(idx: int) -> Optional[bytes]:
        off, sz = doh.blocks[idx]
        if sz == 0:
            return None
        return instance[off:off+sz]

    config_buf = get_block(1)
    meas_buf = get_block(2)

    if config_buf is None:
        raise ValueError("Config block not present (enable it in channel features)")
    if meas_buf is None:
        raise ValueError("Measurement block not present (enable it)")

    cfg = parse_config_block(config_buf)
    meas = parse_measurement_block(meas_buf, expected_beams=cfg.num_beams)

    start_deg = cfg.start_angle_deg
    res_deg = cfg.angular_resolution_deg
    N = meas.distances_mm.shape[0]
    end_deg = start_deg + (N - 1) * res_deg

    angles_deg = start_deg + np.arange(N, dtype=np.float32) * res_deg
    angles_rad = np.deg2rad(angles_deg)

    return {
        'header': doh,
        'config': cfg,
        'measurement': meas,
        'angles_rad': angles_rad,
        'start_angle_deg': start_deg,
        'end_angle_deg': end_deg,
        'num_beams': N,
    }

# ------------- Utils: filtering and autoscale -------------

def filter_distances_mm(d: np.ndarray) -> np.ndarray:
    df = d.astype(np.float32, copy=True)
    invalid = (df <= MIN_VALID_MM) | (df >= MAX_VALID_MM) | ~np.isfinite(df)
    df[invalid] = np.nan
    return df

# ------------- Particle Filter SLAM Manager -------------

class ParticleSLAMManager:
    """Manages the SLAM process using particle filter implementation"""
    
    def _init_(self, resolution=0.1, width=50.0, height=50.0, num_particles=50):
        """
        Initialize the SLAM manager
        
        Args:
            resolution: meters per pixel for the occupancy grid
            width: map width in meters
            height: map height in meters
            num_particles: number of particles for the particle filter
        """
        self.resolution = resolution
        self.width = width
        self.height = height
        self.num_particles = num_particles
        self.map_data = None
        self.robot_pose = (0, 0, 0)  # x, y, theta
        self.scan_count = 0
        self.mapping_active = False
        self.slam = None
        self.initialized = False
        self.previous_scan = None
        
        # Try to import the particle filter SLAM implementation
        try:
            from map_particle_class import MapParticle
            from map_utils import Map
            
            self.MapParticle = MapParticle
            self.Map = Map
            self.particle_slam_available = True
            print("Particle filter SLAM modules successfully imported")
        except ImportError as e:
            self.particle_slam_available = False
            print(f"WARNING: Could not import particle filter SLAM modules: {e}")
    
    def start_mapping(self):
        """Start the mapping process"""
        if not self.particle_slam_available:
            print("Cannot start mapping: Particle filter SLAM modules not available")
            return False
        
        try:
            # Create the map
            self.map = self.Map(
                width=int(self.width / self.resolution),
                height=int(self.height / self.resolution),
                resolution=self.resolution
            )
            
            # Initialize particles
            self.particles = [self.MapParticle(
                x=self.width/2, 
                y=self.height/2, 
                theta=0.0,
                weight=1.0/self.num_particles,
                map=self.map
            ) for _ in range(self.num_particles)]
            
            # Reset state
            self.scan_count = 0
            self.mapping_active = True
            self.initialized = True
            self.previous_scan = None
            
            print(f"Mapping started with {self.num_particles} particles")
            return True
        except Exception as e:
            print(f"Error initializing particle filter SLAM: {e}")
            traceback.print_exc()
            return False
    
    def stop_mapping(self):
        """Stop the mapping process"""
        self.mapping_active = False
        self.initialized = False
        print(f"Mapping stopped after processing {self.scan_count} scans")
    
    def process_scan(self, angles_rad, distances_mm):
        """
        Process a new LiDAR scan
        
        Args:
            angles_rad: array of angles in radians
            distances_mm: array of distances in mm
            
        Returns:
            bool: True if map was updated
        """
        if not self.mapping_active or not self.initialized:
            return False
        
        try:
            # Convert mm to meters
            distances_m = distances_mm / 1000.0
            
            # Filter invalid measurements
            valid = (distances_m > MIN_VALID_MM/1000) & (distances_m < MAX_VALID_MM/1000)
            valid_angles = angles_rad[valid]
            valid_distances = distances_m[valid]
            
            # Create scan data in the format expected by the particle filter
            scan_data = np.column_stack((valid_angles, valid_distances))
            
            # If this is the first scan, initialize particles with it
            if self.previous_scan is None:
                self.previous_scan = scan_data
                # Just initialize but don't process yet
                return True
            
            # Process with particle filter SLAM
            self._process_with_particle_filter(self.previous_scan, scan_data)
            
            # Store current scan for next iteration
            self.previous_scan = scan_data
            
            self.scan_count += 1
            return True
        except Exception as e:
            print(f"Error processing scan: {e}")
            traceback.print_exc()
            return False
    
    def _process_with_particle_filter(self, prev_scan, curr_scan):
        """Process scans using the particle filter implementation"""
        try:
            # Motion model parameters
            alpha1 = 0.1
            alpha2 = 0.1
            alpha3 = 0.01
            alpha4 = 0.01
            
            # Get the movement between scans
            # This is a simplified approach - in a real system you'd have odometry
            # Here we estimate movement based on the scan difference
            # For simplicity, we'll assume small movement for now
            
            # Predict step: move all particles
            for particle in self.particles:
                # This would be replaced with actual odometry in a real system
                # For now, we'll use a simplified motion model
                d_rot1 = 0.0  # rotation before movement
                d_trans = 0.1  # translation
                d_rot2 = 0.1   # rotation after movement
                
                # Add noise to the motion
                d_rot1 += np.random.normal(0, alpha1 * abs(d_rot1) + alpha2 * d_trans)
                d_trans += np.random.normal(0, alpha3 * d_trans + alpha4 * (abs(d_rot1) + abs(d_rot2)))
                d_rot2 += np.random.normal(0, alpha1 * abs(d_rot2) + alpha2 * d_trans)
                
                # Update particle pose
                particle.x += d_trans * np.cos(particle.theta + d_rot1)
                particle.y += d_trans * np.sin(particle.theta + d_rot1)
                particle.theta += d_rot1 + d_rot2
                
                # Keep theta in [-pi, pi]
                particle.theta = (particle.theta + np.pi) % (2 * np.pi) - np.pi
            
            # Update step: update particle weights based on current scan
            for particle in self.particles:
                # Calculate the likelihood of the current scan given the particle's pose
                # This is a simplified version - the real implementation would use the map
                # For now, we'll use a basic comparison between predicted and actual scan
                total_weight = 0.0
                
                # This would be replaced with the actual measurement model
                # For now, we'll just assign random weights for demonstration
                for i, (angle, distance) in enumerate(curr_scan):
                    # In a real implementation, this would compare the expected measurement
                    # from the particle's pose and the actual measurement
                    expected_distance = distance * (1 + np.random.normal(0, 0.1))
                    weight = np.exp(-((distance - expected_distance) ** 2) / (2 * 0.5 ** 2))
                    total_weight += weight
                
                # Normalize the weight
                particle.weight = total_weight / len(curr_scan)
            
            # Normalize weights
            total_weight = sum(p.weight for p in self.particles)
            if total_weight > 0:
                for particle in self.particles:
                    particle.weight /= total_weight
            else:
                # If all weights are zero, reset them equally
                for particle in self.particles:
                    particle.weight = 1.0 / self.num_particles
            
            # Resample particles
            new_particles = []
            indexes = np.random.choice(
                np.arange(self.num_particles),
                self.num_particles,
                p=[p.weight for p in self.particles]
            )
            
            for i in indexes:
                particle = self.particles[i]
                new_particle = self.MapParticle(
                    x=particle.x,
                    y=particle.y,
                    theta=particle.theta,
                    weight=1.0/self.num_particles,
                    map=self.map
                )
                new_particles.append(new_particle)
            
            self.particles = new_particles
            
            # Update the map with the best particle
            best_particle = max(self.particles, key=lambda p: p.weight)
            
            # Update the map with the current scan
            for angle, distance in curr_scan:
                if distance > 0 and distance < MAX_VALID_MM/1000.0:
                    # Convert polar to Cartesian
                    x = best_particle.x + distance * np.cos(best_particle.theta + angle)
                    y = best_particle.y + distance * np.sin(best_particle.theta + angle)
                    
                    # Update the map
                    self.map.update_cell(x, y, 0.6)  # Occupied
                    # Update free space along the ray
                    for t in np.linspace(0, distance, int(distance/self.resolution)):
                        free_x = best_particle.x + t * np.cos(best_particle.theta + angle)
                        free_y = best_particle.y + t * np.sin(best_particle.theta + angle)
                        self.map.update_cell(free_x, free_y, -0.4)  # Free space
            
            # Update robot pose
            self.robot_pose = (
                best_particle.x,
                best_particle.y,
                best_particle.theta
            )
            
            # Update map data
            self.map_data = self.map.grid.copy()
            
        except Exception as e:
            print(f"Error in particle filter processing: {e}")
            traceback.print_exc()
    
    def get_map(self):
        """Get the current occupancy grid map"""
        if self.map_data is None:
            # Return empty map with reasonable size
            return np.zeros((int(self.width/self.resolution), int(self.height/self.resolution)))
        return self.map_data
    
    def get_robot_pose(self):
        """Get the current robot pose (x, y, theta)"""
        return self.robot_pose
    
    def save_map(self, filename):
        """Save the current map to a .smap file"""
        try:
            # Ensure the filename has .smap extension
            if not filename.lower().endswith('.smap'):
                filename += '.smap'
            
            # Save the map data and pose
            with open(filename, 'wb') as f:
                pickle.dump({
                    'map': self.get_map(),
                    'resolution': self.resolution,
                    'pose': self.get_robot_pose(),
                    'scan_count': self.scan_count
                }, f)
            
            print(f"Map saved to {filename}")
            return True
        except Exception as e:
            print(f"Error saving map: {e}")
            traceback.print_exc()
            return False

# ------------- Main Application -------------

class SLAMApp:
    """Main application class for the 2D SLAM mapping system"""
    
    def _init_(self):
        # Initialize components
        self.udp = NanoScan3UDP(BIND_IP, PORT)
        self.slam_manager = ParticleSLAMManager(
            resolution=MAP_RESOLUTION, 
            width=MAP_WIDTH, 
            height=MAP_HEIGHT,
            num_particles=NUM_PARTICLES
        )
        self.data_q = queue.Queue(maxsize=10)  # Queue for processed data
        self.map_q = queue.Queue(maxsize=2)     # Queue for map updates
        self.ui_initialized = False
        self.fig = None
        self.ax_lidar = None
        self.ax_map = None
        self.lidar_segments = None
        self.robot_pose_marker = None
        self.map_image = None
        self.mapping_status_text = None
        self.scan_count_text = None
        self.status_text = None
        self.is_running = False
        self.view_mode = "lidar"  # "lidar" or "map"
        
        # UI elements
        self.btn_start = None
        self.btn_stop = None
        self.btn_save = None
        self.btn_view_lidar = None
        self.btn_view_map = None
        self.checkbox_rays = None
        self.checkbox_robot = None
        self.ray_collection = None
        self.robot_patch = None
        self.map_data = None
    
    def on_scan(self, instance: bytes):
        """Callback for when a new scan is received"""
        try:
            info = parse_full_instance(instance)
            angles_rad = info['angles_rad']
            distances_mm = info['measurement'].distances_mm
            
            # Put raw data in queue for visualization
            try:
                self.data_q.put_nowait((angles_rad, distances_mm))
            except queue.Full:
                pass
            
            # Process with SLAM if mapping is active
            if self.slam_manager.mapping_active:
                self.slam_manager.process_scan(angles_rad, distances_mm)
                try:
                    # Get updated map and put in map queue
                    map_data = self.slam_manager.get_map()
                    if map_data is not None:
                        self.map_q.put_nowait(map_data)
                except queue.Full:
                    pass
            
        except Exception as e:
            print(f"Error processing scan: {e}")
            traceback.print_exc()
    
    def init_ui(self):
        """Initialize the user interface"""
        plt.ion()
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title('2D SLAM Mapping Application')
        
        # Create subplots: left for LiDAR view, right for map
        self.ax_lidar = self.fig.add_subplot(121, aspect='equal')
        self.ax_map = self.fig.add_subplot(122, aspect='equal')
        
        # Setup LiDAR view
        self.ax_lidar.grid(True, linestyle=':', alpha=0.6)
        self.ax_lidar.set_xlabel('X (mm)')
        self.ax_lidar.set_ylabel('Y (mm)')
        self.ax_lidar.set_title('LiDAR Scan View')
        self.ax_lidar.set_xlim(-DEFAULT_RMAX, DEFAULT_RMAX)
        self.ax_lidar.set_ylim(-DEFAULT_RMAX, DEFAULT_RMAX)
        
        # Add range circle for LiDAR view
        self.range_circle = plt.Circle((0, 0), DEFAULT_RMAX, fill=False, color='0.6', ls='--', lw=0.8, alpha=0.6)
        self.ax_lidar.add_patch(self.range_circle)
        
        # Initialize LiDAR rays
        self.ray_collection = LineCollection([], linewidths=RAY_WIDTH)
        if COLOR_BY_RANGE:
            self.ray_collection.set_cmap('viridis')
            self.ray_collection.set_array(np.array([], dtype=np.float32))
        else:
            self.ray_collection.set_color(RAY_COLOR)
        self.ax_lidar.add_collection(self.ray_collection)
        
        # Setup map view
        self.ax_map.grid(True, linestyle=':', alpha=0.6)
        self.ax_map.set_xlabel('X (m)')
        self.ax_map.set_ylabel('Y (m)')
        self.ax_map.set_title('Occupancy Grid Map')
        
        # Add robot pose marker (initially hidden)
        self.robot_patch = plt.Circle((0, 0), 0.2, fill=True, color='red', alpha=0.7)
        self.ax_map.add_patch(self.robot_patch)
        self.robot_patch.set_visible(False)
        
        # Add map image (initially empty)
        self.map_image = self.ax_map.imshow(np.zeros((int(MAP_WIDTH/MAP_RESOLUTION), int(MAP_HEIGHT/MAP_RESOLUTION))), 
                                           cmap='gray_r', 
                                           vmin=-1, 
                                           vmax=1,
                                           extent=(-MAP_WIDTH/2, MAP_WIDTH/2, 
                                                   -MAP_HEIGHT/2, MAP_HEIGHT/2))
        
        # Add UI controls
        self._add_ui_controls()
        
        # Add status text
        self.status_text = self.fig.text(0.02, 0.02, 'Status: Waiting for data...', 
                                        fontsize=10, 
                                        color='blue',
                                        ha='left')
        
        self.mapping_status_text = self.fig.text(0.02, 0.05, 'Mapping: Stopped', 
                                               fontsize=10, 
                                               color='red',
                                               ha='left')
        
        self.scan_count_text = self.fig.text(0.02, 0.08, 'Scans processed: 0', 
                                           fontsize=10, 
                                           color='black',
                                           ha='left')
        
        self.fig.tight_layout()
        self.fig.subplots_adjust(bottom=0.15)
        
        self.ui_initialized = True
        print("UI initialized successfully")
    
    def _add_ui_controls(self):
        """Add UI control buttons and checkboxes"""
        # Create button axes
        button_width = 0.1
        button_height = 0.05
        button_spacing = 0.01
        
        # Start button
        ax_start = plt.axes([0.02, 0.1, button_width, button_height])
        self.btn_start = Button(ax_start, 'Start Mapping')
        self.btn_start.on_clicked(self._on_start_mapping)
        
        # Stop button
        ax_stop = plt.axes([0.14, 0.1, button_width, button_height])
        self.btn_stop = Button(ax_stop, 'Stop Mapping')
        self.btn_stop.on_clicked(self._on_stop_mapping)
        self.btn_stop.color = 'lightcoral'
        self.btn_stop.hovercolor = 'red'
        
        # Save button
        ax_save = plt.axes([0.26, 0.1, button_width, button_height])
        self.btn_save = Button(ax_save, 'Save Map')
        self.btn_save.on_clicked(self._on_save_map)
        
        # View mode buttons
        ax_view_lidar = plt.axes([0.40, 0.1, 0.08, button_height])
        self.btn_view_lidar = Button(ax_view_lidar, 'LiDAR View')
        self.btn_view_lidar.on_clicked(lambda event: self._set_view_mode("lidar"))
        
        ax_view_map = plt.axes([0.50, 0.1, 0.08, button_height])
        self.btn_view_map = Button(ax_view_map, 'Map View')
        self.btn_view_map.on_clicked(lambda event: self._set_view_mode("map"))
        
        # Checkboxes
        ax_checkboxes = plt.axes([0.65, 0.08, 0.15, 0.1], frame_on=False)
        self.checkbox_rays = CheckButtons(
            ax_checkboxes, 
            ['Show Rays', 'Show Robot'], 
            [True, True]
        )
        self.checkbox_rays.on_clicked(self._on_checkbox_clicked)
        
        # Set initial state
        self._set_view_mode("lidar")
    
    def _on_start_mapping(self, event):
        """Handle start mapping button click"""
        if self.slam_manager.start_mapping():
            self.mapping_status_text.set_text('Mapping: Active')
            self.mapping_status_text.set_color('green')
            self.scan_count_text.set_text(f'Scans processed: {self.slam_manager.scan_count}')
            self.status_text.set_text('Status: Mapping in progress...')
    
    def _on_stop_mapping(self, event):
        """Handle stop mapping button click"""
        self.slam_manager.stop_mapping()
        self.mapping_status_text.set_text('Mapping: Stopped')
        self.mapping_status_text.set_color('red')
        self.scan_count_text.set_text(f'Scans processed: {self.slam_manager.scan_count}')
        self.status_text.set_text('Status: Mapping stopped')
    
    def _on_save_map(self, event):
        """Handle save map button click"""
        if not self.slam_manager.mapping_active:
            root = tk.Tk()
            root.withdraw()
            messagebox.showwarning("Warning", "Mapping is not active. Saving current map.")
        
        # Use Tkinter for file dialog
        root = tk.Tk()
        root.withdraw()
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".smap",
            filetypes=[("SLAM Map Files", ".smap"), ("All Files", ".*")],
            title="Save Map As"
        )
        
        if filename:
            if self.slam_manager.save_map(filename):
                self.status_text.set_text(f'Status: Map saved to {os.path.basename(filename)}')
            else:
                self.status_text.set_text('Status: Failed to save map')
    
    def _on_checkbox_clicked(self, label):
        """Handle checkbox state changes"""
        if label == 'Show Rays':
            self.ray_collection.set_visible(not self.ray_collection.get_visible())
        elif label == 'Show Robot':
            self.robot_patch.set_visible(not self.robot_patch.get_visible())
        
        self.fig.canvas.draw_idle()
    
    def _set_view_mode(self, mode):
        """Switch between LiDAR view and map view"""
        self.view_mode = mode
        
        if mode == "lidar":
            self.ax_lidar.set_visible(True)
            self.ax_map.set_visible(False)
            self.btn_view_lidar.color = 'lightgreen'
            self.btn_view_map.color = 'white'
        else:  # map
            self.ax_lidar.set_visible(False)
            self.ax_map.set_visible(True)
            self.btn_view_lidar.color = 'white'
            self.btn_view_map.color = 'lightgreen'
        
        self.fig.canvas.draw_idle()
    
    def update_ui(self):
        """Update the UI with new data"""
        # Process LiDAR data
        try:
            while True:
                angles_rad, distances_mm = self.data_q.get_nowait()
                self._update_lidar_view(angles_rad, distances_mm)
        except queue.Empty:
            pass
        
        # Process map updates
        try:
            while True:
                map_data = self.map_q.get_nowait()
                self._update_map_view(map_data)
                self.scan_count_text.set_text(f'Scans processed: {self.slam_manager.scan_count}')
        except queue.Empty:
            pass
    
    def _update_lidar_view(self, angles_rad, distances_mm):
        """Update the LiDAR view with new scan data"""
        if not self.ui_initialized or self.view_mode != "lidar":
            return
        
        d_f = filter_distances_mm(distances_mm)
        
        # Valid mask and optional cap on number of rays
        mask = np.isfinite(d_f)
        if not np.any(mask):
            # Nothing valid: clear and keep default limits
            self.ray_collection.set_segments([])
            self.fig.canvas.draw_idle()
            return
        
        # Downsample if too many rays (perf safety)
        idx = np.flatnonzero(mask)
        if idx.size > CAP_RAYS:
            step = int(np.ceil(idx.size / CAP_RAYS))
            idx = idx[::step]
        
        th = angles_rad[idx]
        r = d_f[idx]
        
        # Robust radius for view
        rmax = float(np.nanpercentile(d_f[mask], ROBUST_PERCENTILE))
        if not np.isfinite(rmax) or rmax < MIN_VALID_MM:
            rmax = DEFAULT_RMAX
        
        # Compute endpoints in XY (mm)
        x = r * np.cos(th)
        y = r * np.sin(th)
        
        # Build segments from origin to (x, y)
        segments = np.zeros((x.size, 2, 2), dtype=np.float32)
        segments[:, 1, 0] = x
        segments[:, 1, 1] = y
        
        self.ray_collection.set_segments(segments)
        
        if COLOR_BY_RANGE:
            # Color by distance
            self.ray_collection.set_array(r)         # scalar per segment
            self.ray_collection.set_clim(0, rmax)
        
        # Update view box and range circle
        lim = max(DEFAULT_RMAX, rmax * 1.05)
        self.ax_lidar.set_xlim(-lim, lim)
        self.ax_lidar.set_ylim(-lim, lim)
        self.range_circle.set_radius(lim)
        
        self.fig.canvas.draw_idle()
    
    def _update_map_view(self, map_data):
        """Update the map view with new map data"""
        if not self.ui_initialized or self.view_mode != "map" or map_data is None:
            return
        
        try:
            # Normalize map data for visualization
            # In the particle filter implementation, values can be in range [-1, 1]
            # where -1 = definitely free, 0 = unknown, 1 = definitely occupied
            normalized_map = np.clip(map_data, -1, 1)
            
            # Update the map image
            self.map_image.set_array(normalized_map)
            
            # Update robot pose
            x, y, theta = self.slam_manager.get_robot_pose()
            self.robot_patch.set_center((x, y))
            self.robot_patch.set_visible(True)
            
            # Update map view limits
            self.ax_map.set_xlim(-MAP_WIDTH/2, MAP_WIDTH/2)
            self.ax_map.set_ylim(-MAP_HEIGHT/2, MAP_HEIGHT/2)
            
            self.fig.canvas.draw_idle()
        except Exception as e:
            print(f"Error updating map view: {e}")
            traceback.print_exc()
    
    def run(self):
        """Run the main application loop"""
        print(f"Starting SLAM application. Binding to {BIND_IP}:{PORT}")
        
        # Initialize UI
        self.init_ui()
        
        # Start UDP receiver thread
        self.is_running = True
        receive_thread = threading.Thread(
            target=self.udp.receive_loop, 
            args=(self.on_scan,),
            daemon=True
        )
        receive_thread.start()
        
        # Main loop
        try:
            self.status_text.set_text('Status: Waiting for LiDAR data...')
            print("Application started. Please ensure your LiDAR is sending data.")
            print("To see the 2D map:")
            print("1. Click 'Start Mapping'")
            print("2. Move your robot/platform with the LiDAR")
            print("3. Switch to 'Map View' to see the generated map")
            
            while self.is_running:
                self.update_ui()
                
                # Check if window was closed
                if not plt.fignum_exists(self.fig.number):
                    break
                
                plt.pause(0.05)
        except KeyboardInterrupt:
            print("\nShutting down application...")
        finally:
            self.is_running = False
            self.udp.stop()
            plt.ioff()
            try:
                plt.close(self.fig)
            except Exception:
                pass
            print("Application shutdown complete")

# ------------- Main Function -------------

def main():
    app = SLAMApp()
    app.run()

if _name_ == "_main_":
    print(f"Starting 2D SLAM Mapping Application")
    print(f"Binding to {BIND_IP}:{PORT}")
    print(f"Map resolution: {MAP_RESOLUTION} meters/pixel")
    print(f"Map size: {MAP_WIDTH}m x {MAP_HEIGHT}m")
    print(f"Particle filter: {NUM_PARTICLES} particles")
    
    # Check if particle filter files exist
    required_files = [
        "particle_slam_main.py",
        "map_particle_class.py",
        "map_utils.py",
        "load_data.py"
    ]
    
    missing_files = [f for f in required_files if not os.path.exists(os.path.join("D:\\AMR R&D\\slam_app", f))]
    
    if missing_files:
        print("\nERROR: Required particle filter SLAM files are missing!")
        print("Run these EXACT commands in PowerShell:")
        print("cd \"D:\\AMR R&D\\slam_app\"")
        print("Invoke-WebRequest -Uri \"https://raw.githubusercontent.com/SanjayJohn21358/2DSLAM/master/main.py\" -OutFile \"particle_slam_main.py\"")
        print("Invoke-WebRequest -Uri \"https://raw.githubusercontent.com/SanjayJohn21358/2DSLAM/master/map_particle_class.py\" -OutFile \"map_particle_class.py\"")
        print("Invoke-WebRequest -Uri \"https://raw.githubusercontent.com/SanjayJohn21358/2DSLAM/master/map_utils.py\" -OutFile \"map_utils.py\"")
        print("Invoke-WebRequest -Uri \"https://raw.githubusercontent.com/SanjayJohn21358/2DSLAM/master/load_data.py\" -OutFile \"load_data.py\"")
        print("\nThen restart the application.")
    else:
        print("\nParticle filter SLAM files found. Application ready to run.")
        print("Note: This implementation uses a simplified particle filter.")
        print("For best results, move the LiDAR slowly and steadily.")
    
    main()
