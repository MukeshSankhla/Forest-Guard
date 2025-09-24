from flask import Flask, render_template, request, jsonify, send_from_directory
import os, time, json, wave, serial, struct
import serial.tools.list_ports
import requests
from io import BytesIO
from PIL import Image

app = Flask(__name__)
UPLOAD_DIR = 'captured_data'
os.makedirs(UPLOAD_DIR, exist_ok=True)

# ===== Helpers =====
def get_com_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

@app.route('/')
def home():
    # You can build a simple HTML UI that posts to /capture and /record_audio
    return render_template('index.html', com_ports=get_com_ports())

# ===== Image capture (unchanged protocol, small fixes) =====
@app.route('/capture', methods=['POST'])
def capture_image():
    try:
        port = request.form['com_port']
        api_key = request.form['api_key']
        label = request.form.get('label', '')
        mode = request.form['mode']

        upload_url = (
            'https://ingestion.edgeimpulse.com/api/training/files'
            if mode == 'training' else
            'https://ingestion.edgeimpulse.com/api/testing/files'
        )

        with serial.Serial(port, 921600, timeout=10) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b'CAPTURE\n')
            size_line = ser.readline().decode(errors='ignore').strip()

            if not size_line.isdigit():
                raise ValueError(f"Invalid image size: {size_line}")
            size = int(size_line)
            image_data = ser.read(size)

        img = Image.open(BytesIO(image_data))
        img_path = os.path.join(UPLOAD_DIR, 'captured_image.jpg')
        img.save(img_path, 'JPEG')

        files = [('data', ('image.jpg', open(img_path, 'rb'), 'image/jpeg'))]
        if label:
            metadata = {
                "version": 1,
                "type": "bounding-box-labels",
                "boundingBoxes": {"image.jpg": [{
                    "label": label, "x": 0, "y": 0, "width": img.width, "height": img.height
                }]}
            }
            files.append(('data', ('bounding_boxes.labels', json.dumps(metadata), 'application/json')))

        headers = {'x-api-key': api_key}
        try:
            response = requests.post(upload_url, headers=headers, files=files, timeout=30)
        finally:
            # close file handles
            for _, (fn, fh, _) in files:
                try:
                    fh.close()
                except Exception:
                    pass

        return jsonify({
            'status': 'success' if response.status_code == 200 else 'error',
            'message': response.text,
            'image_url': '/static/captured_image.jpg'
        })

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

# ===== Audio capture (binary, exact length) =====
@app.route('/record_audio', methods=['POST'])
def record_audio():
    try:
        import random, string
        port = request.form['com_port']
        api_key = request.form['api_key']
        label = request.form['label']
        mode = request.form['mode']
        sample_rate = 16000

        upload_url = (
            'https://ingestion.edgeimpulse.com/api/training/files'
            if mode == 'training' else
            'https://ingestion.edgeimpulse.com/api/testing/files'
        )

        # Local WAV path & upload filename
        local_wav = os.path.join(UPLOAD_DIR, 'captured_audio.wav')
        upload_name = ''.join(random.choices(string.ascii_lowercase + string.digits, k=8)) + '.wav'

        # Serial communication (binary protocol)
        with serial.Serial(port, 921600, timeout=15) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b'CAPTURE_AUDIO\n')

            header = ser.readline().decode('ascii', errors='ignore').strip()
            if not header.startswith('START_AUDIO'):
                raise RuntimeError(f'Unexpected header from device: {header}')

            parts = header.split()
            expected_bytes = int(parts[1]) if len(parts) == 2 and parts[1].isdigit() else sample_rate * 10 * 2

            # Read exact number of bytes
            audio_bytes = bytearray()
            while len(audio_bytes) < expected_bytes:
                chunk = ser.read(expected_bytes - len(audio_bytes))
                if not chunk:
                    break
                audio_bytes += chunk

            end_line = ser.readline().decode('ascii', errors='ignore').strip()
            if end_line != 'END_AUDIO':
                # Not fatal; we proceed with what we received
                pass

        # Write WAV as 16‑bit little‑endian PCM mono
        with wave.open(local_wav, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(audio_bytes)

        # Build metadata for Edge Impulse
        num_samples = len(audio_bytes) // 2
        metadata = {
            "version": 1,
            "type": "structured-labels",
            "structuredLabels": {
                upload_name: [{
                    "startIndex": 0,
                    "endIndex": max(0, num_samples - 1),
                    "label": label
                }]
            }
        }

        files = [
            ('data', (upload_name, open(local_wav, 'rb'), 'audio/wav')),
            ('data', ('structured_labels.labels', BytesIO(json.dumps(metadata).encode('utf-8')), 'application/json'))
        ]
        headers = {'x-api-key': api_key}
        try:
            response = requests.post(upload_url, headers=headers, files=files, timeout=60)
        finally:
            for _, (fn, fh, _) in files:
                try:
                    fh.close()
                except Exception:
                    pass

        return jsonify({
            'status': 'success' if response.status_code == 200 else 'error',
            'message': response.text,
            'audio_url': '/static/captured_audio.wav',
            'received_samples': num_samples
        })

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/static/<filename>')
def serve_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)

if __name__ == '__main__':
    app.run(debug=True)