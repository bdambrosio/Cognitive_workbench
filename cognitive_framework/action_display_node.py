#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
import sys
from collections import deque
import threading
import queue

try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.text import Text
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False

class ActionDisplayNode(Node):
    def __init__(self):
        super().__init__('action_display_node')
        
        if not RICH_AVAILABLE:
            self.get_logger().error('❌ Rich library not available. Install with: pip install rich')
            return
        
        # Rich console setup
        self.console = Console()
        
        # Action history buffer (keep last 50 actions)
        self.action_history = deque(maxlen=50)
        self.stats = {
            'total_actions': 0,
            'llm_actions': 0,
            'sensor_actions': 0,
            'start_time': datetime.now()
        }
        
        # Subscribe to action data
        self.action_subscriber = self.create_subscription(
            String,
            '/cognitive/action_data',
            self.action_callback,
            10
        )
        
        # Also subscribe to LLM responses to show the full flow
        self.llm_subscriber = self.create_subscription(
            String,
            '/cognitive/llm_response',
            self.llm_response_callback,
            10
        )
        
        # Subscribe to sense data to show input
        self.sense_subscriber = self.create_subscription(
            String,
            '/cognitive/sense_data',
            self.sense_callback,
            10
        )
        
        # Publisher for text input (so users can type in this terminal)
        self.text_input_publisher = self.create_publisher(
            String,
            '/cognitive/text_input',
            10
        )
        
        # Text input handling
        self.text_input_queue = queue.Queue()
        self.awaiting_input = False
        self.last_sent_input = None  # Track last input to prevent duplicates
        self.last_sent_time = None
        
        self.get_logger().info('🎨 Action Display Node initialized - starting rich terminal...')
        self.get_logger().info('💡 You can type directly in this terminal to send input to the cognitive system!')
        
        # Start the live display
        self.start_display()
    
    def action_callback(self, msg):
        """Handle incoming action data"""
        try:
            action_data = json.loads(msg.data)
            timestamp = datetime.now().strftime('%H:%M:%S')
            
            # Add to history
            display_entry = {
                'timestamp': timestamp,
                'type': 'action',
                'data': action_data
            }
            self.action_history.append(display_entry)
            
            # Update stats
            self.stats['total_actions'] += 1
            action_type = action_data.get('action_type', 'unknown')
            if 'llm' in action_type.lower():
                self.stats['llm_actions'] += 1
            else:
                self.stats['sensor_actions'] += 1
                
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse action data JSON')
    
    def llm_response_callback(self, msg):
        """Handle LLM responses to show in the flow"""
        try:
            llm_data = json.loads(msg.data)
            timestamp = datetime.now().strftime('%H:%M:%S')
            
            display_entry = {
                'timestamp': timestamp,
                'type': 'llm_response',
                'data': llm_data
            }
            self.action_history.append(display_entry)
            
            # Update LLM stats counter
            self.stats['llm_actions'] += 1
            
        except json.JSONDecodeError:
            pass
    
    def sense_callback(self, msg):
        """Handle sense data to show text input"""
        try:
            sense_data = json.loads(msg.data)
            
            # Only show console text input
            console_sensor = sense_data.get('data', {}).get('console_text_sensor', {})
            if console_sensor.get('new_input', False):
                timestamp = datetime.now().strftime('%H:%M:%S')
                
                display_entry = {
                    'timestamp': timestamp,
                    'type': 'text_input',
                    'data': {
                        'input': console_sensor.get('current_input', ''),
                        'sensor_type': sense_data.get('sensor_type', 'unknown')
                    }
                }
                self.action_history.append(display_entry)
                
        except json.JSONDecodeError:
            pass
    
    def send_text_input(self, text):
        """Send text input to the cognitive system"""
        if text.strip():
            current_time = datetime.now()
            
            # Check for duplicate input (same text within 2 seconds)
            if (self.last_sent_input == text.strip() and 
                self.last_sent_time and 
                (current_time - self.last_sent_time).total_seconds() < 2.0):
                self.get_logger().warning(f'🚫 Duplicate input detected, ignoring: "{text.strip()}"')
                return
            
            # Update tracking
            self.last_sent_input = text.strip()
            self.last_sent_time = current_time
            
            # Publish to text input topic
            msg = String()
            msg.data = text.strip()
            self.text_input_publisher.publish(msg)
            
            # Add to our own history for immediate feedback
            timestamp = current_time.strftime('%H:%M:%S')
            display_entry = {
                'timestamp': timestamp,
                'type': 'text_input',
                'data': {
                    'input': text.strip(),
                    'sensor_type': 'display_input'
                }
            }
            self.action_history.append(display_entry)
            
            self.get_logger().info(f'📤 Sent text input: "{text.strip()}"')
    

    
    def _input_thread(self):
        """Handle text input in a VSCode-friendly way"""
        try:
            while rclpy.ok():
                try:
                    # Use regular input() which works well in VSCode
                    self.console.print("\n[bold cyan]💬 Enter your message:[/bold cyan]", end=" ")
                    user_input = input()
                    
                    if user_input.strip():
                        self.send_text_input(user_input.strip())
                        self.console.print(f"[green]✅ Sent:[/green]")
                        # Display the full input that was sent with proper wrapping
                        self.console.print(f"    \"{user_input.strip()}\"", style="green italic", width=100)
                        self.console.print()
                    
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self.get_logger().error(f'Input error: {e}')
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Input thread error: {e}')

    def start_display(self):
        """Start the VSCode-friendly scrolling display"""
        # Start input handling thread  
        input_thread = threading.Thread(target=self._input_thread, daemon=True)
        input_thread.start()
        
        self.console.print("\n[bold green]🎨 Action Display Started![/bold green]")
        self.console.print("[bold cyan]💬 The input thread will prompt you for messages[/bold cyan]")
        self.console.print("[dim]" + "="*60 + "[/dim]\n")
        
        last_action_count = 0
        last_display_time = datetime.now()
        
        try:
            while rclpy.ok():
                try:
                    rclpy.spin_once(self, timeout_sec=0.5)
                    
                    # Only refresh display when we have new actions AND enough time has passed
                    current_count = len(self.action_history)
                    current_time = datetime.now()
                    time_since_last_display = (current_time - last_display_time).total_seconds()
                    
                    if current_count != last_action_count and time_since_last_display > 1.0:  # Rate limit to once per second
                        self.print_recent_actions()
                        last_action_count = current_count
                        last_display_time = current_time
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self.get_logger().error(f'Display error: {e}')
                    break
                        
        except KeyboardInterrupt:
            self.console.print("\n[yellow]👋 Action Display stopped by user[/yellow]")
        except Exception as e:
            self.console.print(f"\n[red]❌ Display error: {e}[/red]")
    
    def print_recent_actions(self):
        """Print recent actions in a VSCode-friendly way with full text"""
        # Clear some space
        self.console.print()
        
        # Show stats
        uptime = datetime.now() - self.stats['start_time']
        uptime_str = f"{int(uptime.total_seconds() // 3600):02d}h {int((uptime.total_seconds() % 3600) // 60):02d}m"
        
        stats_line = f"[bold blue]📊 Stats:[/bold blue] Total: {self.stats['total_actions']} | LLM: {self.stats['llm_actions']} | Uptime: {uptime_str}"
        self.console.print(stats_line)
        
        # Show recent actions (last 3 for better readability with full text)
        recent_actions = list(self.action_history)[-3:]
        
        if recent_actions:
            self.console.print("[bold magenta]📝 Recent Actions:[/bold magenta]")
            
            for entry in recent_actions:
                timestamp = entry['timestamp']
                entry_type = entry['type']
                data = entry['data']
                
                if entry_type == 'text_input':
                    input_text = data.get('input', '')
                    self.console.print(f"  [cyan]{timestamp}[/cyan] 🎤 [bold cyan]Input:[/bold cyan]")
                    # Use rich's text wrapping for input
                    self.console.print(f"    \"{input_text}\"", style="italic")
                
                elif entry_type == 'llm_response':
                    response_text = data.get('response', '')
                    request_id = data.get('request_id', 'unknown')
                    self.console.print(f"  [cyan]{timestamp}[/cyan] 🧠 [bold yellow]LLM Response[/bold yellow] [dim](ID: {request_id})[/dim]:")
                    
                    # Display full response with proper wrapping
                    if response_text:
                        # Split into paragraphs and wrap each
                        paragraphs = response_text.split('\n')
                        for paragraph in paragraphs:
                            if paragraph.strip():
                                self.console.print(f"    {paragraph.strip()}", style="white", width=100)
                            else:
                                self.console.print()  # Empty line
                    else:
                        self.console.print("    [dim](empty response)[/dim]")
                
                elif entry_type == 'action':
                    content = data.get('content', '')
                    action_type = data.get('action_type', 'unknown')
                    confidence = data.get('confidence', 0)
                    conf_color = "green" if confidence > 0.8 else "yellow" if confidence > 0.5 else "red"
                    
                    self.console.print(f"  [cyan]{timestamp}[/cyan] 🤖 [bold green]Action:[/bold green] {action_type} [dim]([{conf_color}]Conf: {confidence:.2f}[/{conf_color}])[/dim]")
                    
                    # Display full content with proper handling
                    if isinstance(content, dict):
                        # Pretty print dict content
                        summary = content.get('summary', '')
                        if summary:
                            self.console.print(f"    Summary: {summary}", style="white", width=100)
                        
                        # Show other relevant fields
                        for key, value in content.items():
                            if key != 'summary' and value:
                                self.console.print(f"    {key.title()}: {value}", style="dim", width=100)
                    else:
                        # Display full text content
                        content_str = str(content)
                        if content_str:
                            self.console.print(f"    {content_str}", style="white", width=100)
                        else:
                            self.console.print("    [dim](no content)[/dim]")
                
                # Add separator between entries
                self.console.print()
        
        # Add instructions periodically
        if len(self.action_history) % 10 == 1 and len(self.action_history) > 1:
            self.console.print(f"[dim]💡 Use the input prompts above to send messages • Ctrl+C to exit[/dim]")
        
        self.console.print("[dim]" + "─" * 60 + "[/dim]")

def main(args=None):
    if not RICH_AVAILABLE:
        print("❌ Rich library required. Install with: pip install rich")
        return 1
    
    rclpy.init(args=args)
    
    try:
        node = ActionDisplayNode()
        # The display loop handles spinning
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main()) 