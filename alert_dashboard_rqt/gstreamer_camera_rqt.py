#!/usr/bin/env python3

import os
import sys
import numpy as np
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer, QPoint, QRect, QObject, Signal
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, 
    QHBoxLayout, QLineEdit, QDialog, QFormLayout, QDialogButtonBox, QSizePolicy
)
from python_qt_binding.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont, QBrush

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

# Import bounding box messages - use standard ROS2 vision_msgs
try:
    from vision_msgs.msg import Detection2DArray
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False
    print("Warning: vision_msgs not found. Bounding box overlay disabled.")

# Try importing world_info_msgs for Polygons and Keypoints
try:
    from world_info_msgs.msg import BoundingPolygonArray, KeypointsArray
    HAS_WORLD_INFO_MSGS = True
except ImportError:
    HAS_WORLD_INFO_MSGS = False
    # print("Warning: world_info_msgs not found. Polygon and Keypoint overlays disabled.")

import cv2



class EnlargedImageWindow(QDialog):
    """Separate window to show enlarged image."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Enlarged Camera View (Click to close)")
        self.setMinimumSize(800, 600)
        
        layout = QVBoxLayout()
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setScaledContents(False)
        self.image_label.mousePressEvent = self.on_click
        layout.addWidget(self.image_label)
        self.setLayout(layout)
    
    def on_click(self, event):
        """Close window when clicked."""
        self.close()
    
    def update_pixmap(self, pixmap):
        """Update the displayed image."""
        # Scale to height of 700 while maintaining aspect ratio
        scaled_pixmap = pixmap.scaledToHeight(700, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled_pixmap)


class FrameSignal(QObject):
    """Signal to pass frames from GStreamer thread to Qt thread."""
    new_frame = Signal(object, int, int)  # frame_data, width, height


class SettingsDialog(QDialog):
    """Dialog to configure camera settings."""
    def __init__(self, title, pipeline, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GStreamer Camera Settings")
        self.setMinimumWidth(500)
        
        layout = QVBoxLayout(self)
        form = QFormLayout()
        
        self.title_edit = QLineEdit(title)
        self.pipeline_edit = QLineEdit(pipeline)
        
        form.addRow("Window Title:", self.title_edit)
        form.addRow("GStreamer Pipeline:", self.pipeline_edit)
        
        layout.addLayout(form)
        
        # Add help section for creating a test stream externally
        help_layout = QVBoxLayout()
        help_label = QLabel("<b>How to create a test stream:</b> Run this in your terminal:")
        help_label.setStyleSheet("color: #666;")
        help_layout.addWidget(help_label)

        test_stream_start = "gst-launch-1.0 videotestsrc pattern=smpte ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=5600"
        test_stream_start_edit = QLineEdit(test_stream_start)
        test_stream_start_edit.setReadOnly(True)
        test_stream_start_edit.setStyleSheet("background-color: #e8f4fd; font-family: monospace; font-size: 10px; padding: 5px;")
        help_layout.addWidget(test_stream_start_edit)

        # Add the matching pipeline example for the plugin
        matching_pipeline_label = QLabel("<b>And use this pipeline in the plugin:</b>")
        matching_pipeline_label.setStyleSheet("color: #666; font-size: 10px;")
        help_layout.addWidget(matching_pipeline_label)
        
        matching_pipeline = "udpsrc port=5600 ! application/x-rtp,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink"
        matching_edit = QLineEdit(matching_pipeline)
        matching_edit.setReadOnly(True)
        matching_edit.setStyleSheet("background-color: #e8f4fd; font-family: monospace; font-size: 10px; padding: 5px;")
        help_layout.addWidget(matching_edit)
        
        layout.addLayout(help_layout)
        layout.addSpacing(10)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_settings(self):
        return {
            'title': self.title_edit.text(),
            'pipeline': self.pipeline_edit.text()
        }


class CameraRqtPlugin(Plugin):
    """RQT Plugin to display camera feed from GStreamer pipeline.
    
    Environment Variables:
        CAMERA_PIPELINE: Full GStreamer pipeline string
    
    Examples:
        # Custom pipeline
        export CAMERA_PIPELINE="v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink"
    """
    
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self.context = context
        self.node = context.node
        
        # Default settings
        self.window_title = "GStreamer Camera"
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create the main widget
        self.widget = QWidget()
        layout = QVBoxLayout()
        layout.setSizeConstraint(QVBoxLayout.SetMinimumSize)  # Allow shrinking
        self.widget.setLayout(layout)
        # Allow the widget to be resized freely
        self.widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        
        # Create label to display camera feed
        self.image_label = QLabel("Waiting for camera feed...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: black; color: white; font: bold 16px;")
        self.image_label.setScaledContents(False)
        self.image_label.setMouseTracking(True)
        self.image_label.mousePressEvent = self.on_image_clicked
        # Allow the label to shrink below its content size
        self.image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        
        # Store default pipeline - configurable via environment variables
        self.default_pipeline = os.getenv("CAMERA_PIPELINE")
        if not self.default_pipeline:
            # Default to test pattern if not provided
            self.default_pipeline = "videotestsrc pattern=smpte ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink"
            self.node.get_logger().info("Using default test pattern source")
        else:
            self.node.get_logger().info(f"Using pipeline from CAMERA_PIPELINE")
        
        # Add widgets to layout
        layout.addWidget(self.image_label)
        
        # Add the widget to the plugin
        self._widget = self.widget
        context.add_widget(self._widget)
        
        # GStreamer pipeline
        self.pipeline = None
        self.appsink = None
        
        # Bounding box data
        # Bounding box data
        self.bounding_box_map = {}  # Map of type -> list of detections
        self.bounding_polygon_map = {} # Map of type -> list of polygons
        self.keypoints_map = {} # Map of type -> list of keypoints
        
        self.bb_subscriber = None
        self.bp_subscriber = None
        self.kp_subscriber = None
        
        # Current image for overlay
        self.current_pixmap = None
        
        # Enlarged window
        self.enlarged_window = EnlargedImageWindow()
        
        # Frame counter for status
        self.frame_count = 0
        self.last_frame_time = self.node.get_clock().now()
        
        # Signal to pass frames from GStreamer thread to Qt thread
        self.frame_signal = FrameSignal()
        self.frame_signal.new_frame.connect(self.display_image)
        
        # No auto-start here - restore_settings will handle it
        

    def start_pipeline(self):
        """Start the GStreamer pipeline."""
        try:
            pipeline_str = self.default_pipeline
            self.node.get_logger().info(f"Starting GStreamer pipeline: {pipeline_str}")
            
            # Create pipeline from string
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Get the appsink element
            self.appsink = self.pipeline.get_by_name('sink')
            if not self.appsink:
                raise Exception("Pipeline must contain an appsink named 'sink'")
            
            # CRITICAL: Configure appsink to use callbacks instead of polling
            # Connect to the 'new-sample' signal which fires when a frame is available
            self.appsink.set_property('emit-signals', True)
            self.appsink.set_property('sync', False)
            self.appsink.set_property('max-buffers', 1)
            self.appsink.set_property('drop', True)
            self.appsink.connect('new-sample', self.on_new_sample)
            self.node.get_logger().info("Appsink configured with new-sample callback")
            self.node.get_logger().info(f"Appsink emit-signals: {self.appsink.get_property('emit-signals')}")
            
            # Set pipeline to playing state
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("Unable to set pipeline to PLAYING state")
            
            # Subscribe to bounding box topic if available
            if HAS_VISION_MSGS:
                self.bb_subscriber = self.node.create_subscription(
                    Detection2DArray,
                    "/detections",
                    self.bounding_box_callback,
                    10
                )
                self.node.get_logger().info("Subscribed to /detections for bounding boxes")

            # Subscribe to world_info_msgs topics if available
            if HAS_WORLD_INFO_MSGS:
                self.bp_subscriber = self.node.create_subscription(
                    BoundingPolygonArray,
                    "/bounding_polygons",
                    self.bounding_polygon_callback,
                    10
                )
                self.kp_subscriber = self.node.create_subscription(
                    KeypointsArray,
                    "/keypoints",
                    self.keypoints_callback,
                    10
                )
                self.node.get_logger().info("Subscribed to /bounding_polygons and /keypoints")
            
            # Pipeline started

            self.node.get_logger().info("GStreamer pipeline started successfully")
            
        except Exception as e:
            self.node.get_logger().error(f"Error starting pipeline: {str(e)}")
            # Error occurred

    
    def stop_pipeline(self):
        """Stop the GStreamer pipeline."""
        try:
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.appsink = None
            
            # Unsubscribe from bounding box topic
            # Unsubscribe from topics
            if self.bb_subscriber:
                self.node.destroy_subscription(self.bb_subscriber)
                self.bb_subscriber = None
            if self.bp_subscriber:
                self.node.destroy_subscription(self.bp_subscriber)
                self.bp_subscriber = None
            if self.kp_subscriber:
                self.node.destroy_subscription(self.kp_subscriber)
                self.kp_subscriber = None
            
            self.image_label.setText("Camera stopped")
            # Pipeline stopped

            self.node.get_logger().info("GStreamer pipeline stopped")
            
        except Exception as e:
            self.node.get_logger().error(f"Error stopping pipeline: {str(e)}")

    def bounding_box_callback(self, msg):
        """Callback for vision_msgs Detection2DArray messages."""
        boxes = []
        for detection in msg.detections:
            # Get bounding box from detection
            bbox = detection.bbox
            
            # Get label and confidence
            label = detection.results[0].hypothesis.class_id if detection.results else "unknown"
            confidence = detection.results[0].hypothesis.score if detection.results else 0.0
            
            # Store as center coordinates to match C++ logic
            boxes.append({
                'x': int(bbox.center.position.x),
                'y': int(bbox.center.position.y),
                'width': int(bbox.size_x),
                'height': int(bbox.size_y),
                'text': f"{confidence:.2f}:{label}",
                'time_second': msg.header.stamp.sec
            })
        self.bounding_box_map["detections"] = boxes

    def bounding_polygon_callback(self, msg):
        """Callback for world_info_msgs BoundingPolygonArray."""
        polygons = []
        for bp in msg.array:
            contour = []
            for point in bp.array:
                contour.append([int(point.x), int(point.y)])
            
            text = bp.name
            if bp.confidence > 0:
                text = f"{bp.confidence:.2f}:{bp.name}"

            polygons.append({
                'contour': np.array(contour, dtype=np.int32),
                'text': text,
                'time_second': msg.header.stamp.sec
            })
        self.bounding_polygon_map[msg.type] = polygons

    def keypoints_callback(self, msg):
        """Callback for world_info_msgs KeypointsArray."""
        keypoints_list = []
        for kp in msg.array:
            points = []
            for point in kp.array:
                points.append((int(point.x), int(point.y)))
            
            keypoints_list.append({
                'keypoints': points,
                'time_second': msg.header.stamp.sec
            })
        self.keypoints_map[msg.type] = keypoints_list
    
    
    
    def on_new_sample(self, appsink):
        """Callback when a new sample is available from GStreamer.
        This is called automatically by GStreamer when a frame arrives.
        Runs in GStreamer thread, so we emit a signal to update Qt widgets safely."""
        # self.node.get_logger().info("on_new_sample called!")  # DEBUG
        try:
            # Pull the sample from the appsink
            sample = appsink.pull_sample()
            if not sample:
                return Gst.FlowReturn.OK
            
            # Get the buffer from the sample
            buffer = sample.get_buffer()
            if not buffer:
                return Gst.FlowReturn.OK
                
            caps = sample.get_caps()
            
            # Get video dimensions from caps
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
            
            # Log once when we start receiving frames
            # if self.frame_count == 0:
            #     self.node.get_logger().info(f"Receiving frames: {width}x{height}")
            
            # Extract buffer data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if success:
                # Convert to numpy array - make a copy since we're crossing threads
                frame_data = np.ndarray(
                    shape=(height, width, 3),
                    dtype=np.uint8,
                    buffer=map_info.data
                ).copy()  # IMPORTANT: Copy data before unmapping
                
                buffer.unmap(map_info)
                
                # Emit signal to Qt thread to display the frame
                self.frame_signal.new_frame.emit(frame_data, width, height)
                
                # Update frame counter
                self.frame_count += 1
                current_time = self.node.get_clock().now()
                elapsed = (current_time - self.last_frame_time).nanoseconds / 1e9
                if elapsed >= 1.0:
                    fps = self.frame_count / elapsed
                    # Update status in Qt thread
                    # FPS: {fps:.1f}
                    self.frame_count = 0
                    self.last_frame_time = current_time
            
            return Gst.FlowReturn.OK
                
        except Exception as e:
            self.node.get_logger().error(f"Error processing frame: {str(e)}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            return Gst.FlowReturn.ERROR

    
    def display_image(self, frame_data, width, height):
        """Display frame in Qt label with bounding box overlays using OpenCV."""
        try:
            # We already have frame_data as numpy array (RGB)
            # Make a writeable copy for OpenCV drawing if not already
            if not frame_data.flags.writeable:
                frame_data = frame_data.copy()
            
            conversion_mat_ = frame_data
            current_time_sec = self.node.get_clock().now().to_msg().sec

            # Draw bounding boxes on the 'conversion_mat_' using OpenCV
            for _, bb_array in self.bounding_box_map.items():
                for bb in bb_array:
                    # Ignore if bb data is older than 2 seconds
                    if current_time_sec > bb['time_second'] + 2:
                        continue

                    # Define the bounding box coordinates
                    # bb['x'], bb['y'] are center coordinates
                    x_tl = int(bb['x'] - 0.5 * bb['width'])
                    y_tl = int(bb['y'] - 0.5 * bb['height'])
                    
                    # Draw the bounding box using OpenCV
                    # distinct color (255, 0, 0) - In RGB this is Red. In BGR logic it would be Blue.
                    # Using values from C++ snippet.
                    cv2.rectangle(conversion_mat_, (x_tl, y_tl), (x_tl + bb['width'], y_tl + bb['height']), (255, 0, 0), 4)

                    # Define the text position within the bounding box
                    text_pos = (x_tl + 5, y_tl - 5)

                    # Draw the text inside the bounding box using OpenCV
                    cv2.putText(conversion_mat_, bb['text'], text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Draw polygon on the 'conversion_mat_' using OpenCV
            for _, bp_array in self.bounding_polygon_map.items():
                for bp in bp_array:
                    # Ignore if bp data is older than 2 seconds
                    if current_time_sec > bp['time_second'] + 2:
                        continue

                    pts = bp['contour']
                    # cv2.polylines expects a list of arrays
                    cv2.polylines(conversion_mat_, [pts], True, (255, 0, 0), 1)

                    # Draw the text inside the polygon using OpenCV
                    # Use first point of contour for text position
                    text_pos = tuple(pts[0])
                    cv2.putText(conversion_mat_, bp['text'], text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Draw keypoints on the 'conversion_mat_' using OpenCV
            for _, kp_array in self.keypoints_map.items():
                for kp in kp_array:
                    # Ignore if kp data is older than 2 seconds
                    if current_time_sec > kp['time_second'] + 2:
                        continue

                    # Draw each point as a visible marker
                    for point in kp['keypoints']:
                        cv2.drawMarker(conversion_mat_, point, (255, 0, 0), cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)

            
            bytes_per_line = 3 * width
            
            # Convert to QImage
            # frame_data is modified in place (conversion_mat_)
            q_image = QImage(frame_data.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Create pixmap
            pixmap = QPixmap.fromImage(q_image)
            
            # Store current pixmap for enlarged view
            self.current_pixmap = pixmap
            
            # Scale image to fit label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Display the image
            self.image_label.setPixmap(scaled_pixmap)
            
            # Update enlarged window if visible
            if self.enlarged_window and self.enlarged_window.isVisible():
                self.enlarged_window.update_pixmap(pixmap)
            
        except Exception as e:
            self.node.get_logger().error(f"Error displaying image: {str(e)}")
    
    def on_image_clicked(self, event):
        """Handle mouse click on image to show enlarged view."""
        if self.current_pixmap and event.button() == Qt.LeftButton:
            self.enlarged_window.update_pixmap(self.current_pixmap)
            self.enlarged_window.show()

    def trigger_configuration(self):
        """Open settings dialog."""
        dialog = SettingsDialog(self.window_title, self.default_pipeline, self.widget)
        if dialog.exec_() == QDialog.Accepted:
            settings = dialog.get_settings()
            self.window_title = settings['title']
            self.default_pipeline = settings['pipeline']
            
            # Apply title
            self.widget.setWindowTitle(self.window_title)
            
            # Restart pipeline with new settings
            self.stop_pipeline()
            self.start_pipeline()

    def save_settings(self, plugin_settings, instance_settings):
        """Save settings for persistence."""
        instance_settings.set_value('window_title', self.window_title)
        instance_settings.set_value('pipeline', self.default_pipeline)

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore saved settings."""
        self.window_title = instance_settings.value('window_title', self.window_title)
        self.default_pipeline = instance_settings.value('pipeline', self.default_pipeline)
        
        # Apply title
        self.widget.setWindowTitle(self.window_title)
        
        # Restart pipeline with restored settings
        self.stop_pipeline()
        self.start_pipeline()

    def shutdown_plugin(self):
        """Cleanup when plugin is shut down."""
        self.stop_pipeline()
        if self.enlarged_window:
            self.enlarged_window.close()
