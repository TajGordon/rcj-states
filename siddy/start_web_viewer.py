#!/usr/bin/env python3
"""
Launcher script for the Soccer Bot web viewer.
"""

from soccer_bot_web_server import run_server

if __name__ == "__main__":
    print("🤖 Soccer Bot Web Viewer")
    print("=" * 50)
    print("This will start a web server to view the camera feed")
    print("with real-time ball detection visualization.")
    print()
    
    # Run the web server
    run_server(host='0.0.0.0', port=5000, debug=False)
