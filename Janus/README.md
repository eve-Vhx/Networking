## USAGE

### Create a new stream
```bash python janus_stream.py create <stream_id> <video_port>```

video_port - value between 5000 and 6000. Cannot be the same as the video port of an existing stream.

### Destroy and existing stream
```bash python janus_stream.py destroy <stream_id>```