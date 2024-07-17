import requests
import wave
import io
import pyaudio

DEFAULT ={
    "ref_audio_path": "F:\coding\GPT-SoVITS-Fast\\2.闲聊·聆听_趴在草地上，能听见大地的心跳。.mp3",
    "prompt_text": "趴在草地上，能听见大地的心跳。",
    "prompt_lang": "zh",
    "batch_size": 16,
    "text": "",
    "text_lang": "en",
}


class GPTSoVits:
    def __init__(self, url, verbose=True):
        self.url = url
        self.verbose = verbose

    def send_post(self, data, proxies=None):
        try:
            response = requests.post(self.url, json=data, proxies=proxies)
            response.raise_for_status()  # 如果请求失败，抛出异常

            if response.status_code == 200:
                # 成功：返回音频流
                return response
            else:
                # 失败：返回错误信息
                error_info = response.json()
                if self.verbose:
                    print(f"Error: {error_info}")
                return None

        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
            return None

    def play_audio(self, audio_data):
        wf = wave.open(io.BytesIO(audio_data), 'rb')
        p = pyaudio.PyAudio()

        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True
        )

        data = wf.readframes(1024)
        while data:
            stream.write(data)
            data = wf.readframes(1024)

        stream.stop_stream()
        stream.close()
        p.terminate()
        wf.close()

    def run(self, txt, parameters: dict = None, proxies=None):
        if parameters is None:
            send_data = DEFAULT
        else:
            send_data = parameters
        send_data["text"] = txt
        response = self.send_post(send_data, proxies)
        if response is not None:
            self.play_audio(response.content)
            if self.verbose:
                print("Audio played successfully")
        else:
            print("No audio")



