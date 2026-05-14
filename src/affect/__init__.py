"""Affect: Jill processing-state visualization.

Two pieces:
  - publisher.AffectPublisher: in-process, called from ChatLoop. Publishes
    a small JSON state vector to Zenoh whenever state changes.
  - display/: separate process that subscribes to Zenoh and fans out to a
    P5.js sketch over WebSocket. Launch independently:
        python -m affect.display
    then open src/affect/display/static/index.html in a browser.
"""
