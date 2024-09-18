#!/usr/bin/env python3
import sys
import requests
import json

JANUS_URL = "http://34.230.54.205:8088/janus"
admin_key = "janusoverlord"

def create_session():
    response = requests.post(JANUS_URL, json={
        "janus": "create",
        "transaction": "create-session"
    })
    return response.json()["data"]["id"]

def attach_plugin(session_id):
    response = requests.post(f"{JANUS_URL}/{session_id}", json={
        "janus": "attach",
        "plugin": "janus.plugin.streaming",
        "transaction": "attach-streaming"
    })
    return response.json()["data"]["id"]

def create_stream(stream_id, video_port):
    session_id = create_session()
    handle_id = attach_plugin(session_id)
    
    response = requests.post(f"{JANUS_URL}/{session_id}/{handle_id}", json={
        "janus": "message",
        "body": {
            "request": "create",
            "admin_key": admin_key,
            "type": "rtp",
            "id": int(stream_id),
            "name": f"Stream {stream_id}",
            "description": f"Dynamically created stream {stream_id}",
            "audio": False,
            "video": True,
            "videoport": video_port,
            "videopt": 96,
            "videortpmap": "H264/90000"
        },
        "transaction": f"create-stream-{stream_id}"
    })
    
    result = response.json()
    if result.get("janus") == "success":
        print(f"Stream {stream_id} created successfully")
    else:
        print(f"Failed to create stream {stream_id}")
        print(json.dumps(result, indent=2))

def destroy_stream(stream_id):
    session_id = create_session()
    handle_id = attach_plugin(session_id)
    
    response = requests.post(f"{JANUS_URL}/{session_id}/{handle_id}", json={
        "janus": "message",
        "body": {
            "request": "destroy",
            "id": int(stream_id)
        },
        "transaction": f"destroy-stream-{stream_id}"
    })
    
    result = response.json()
    if result.get("janus") == "success":
        print(f"Stream {stream_id} destroyed successfully")
    else:
        print(f"Failed to destroy stream {stream_id}")
        print(json.dumps(result, indent=2))

def print_usage():
    print("Usage:")
    print("  python janus_stream.py create <stream_id> <video_port>")
    print("  python janus_stream.py destroy <stream_id>")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)
    
    action = sys.argv[1]
    stream_id = sys.argv[2]
    
    if action == "create" and len(sys.argv) == 4:
        video_port = int(sys.argv[3])
        create_stream(stream_id, video_port)
    elif action == "destroy":
        destroy_stream(stream_id)
    else:
        print_usage()
        sys.exit(1)