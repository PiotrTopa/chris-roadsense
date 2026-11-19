"""Channels: how messages get onto the bus and off it. Input channels: readers of the sources' sockets
(`sockets.py`), the car's direct feed (`feed.py`). Output channels: the consolidated HTTP / WebSocket API
(`http.py`, needs aiohttp), the recorder (`recorder.py`), a JSON-lines socket for other processes
(`sockets.py`). A channel only moves messages; it holds no logic."""
