from openai import OpenAI

DEFAULT = """
You are a discriminator, you should determine whether the sentence is idle talk or an
instruction, and return json format data. There are two keys in the json format,
the first is command and the second is content. command: 0 means instruction, command: 1 
means chat, if command is 0 content is your attitude, if command is 1 content is your 
answer. Example 1: Please help me find my cell phone, return {\"command\": 0, \"content\": 
\"OK, I'll try to find it for you \"}, example 2: What's the weather today, 
return {\"command\": 1, \"content\": \"It's a nice day today \"}, Example 3: Can you tell me 
your name, return {\"command\": 1, \"content\":\" Sure, my name is unix, I'm your personal 
assistant \"}, Example 4: I can't find my keys right now, can you help me find them, 
return {\"command\": 0, \"content\": \"Of course \"}, Example 5: OK, you found my key, 
thank you, return {\"command\": 1, \"content\":\" You're welcome \"}
"""


class RobotGPT:
    def __init__(self, api_key="sk-CAaGpNtbbMBoOsLtF5Fa03B782A34937BbC4A25786Aa55Fc", base_url="https://api.xiaoai.plus/v1", model='gpt-3.5-turbo', verbose=True):
        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
        )
        self.model = model
        self.verbose = verbose

    def get_response(self, txt, prompt: list = None, **kwargs):
        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    *prompt,
                    {"role": "user", "content": f"{txt}"},
                ],
                **kwargs
            )
            if self.verbose:
                print(completion)
            return completion
        except Exception as e:
            print(f"Request failed: {e}")
            return []

    def get_command(self, txt, prompt: list = None, **kwargs):
        if prompt is None:
            completion = self.get_response(txt, [{"role": "system", "content": DEFAULT}], **kwargs)
            content = completion.choices[0].message.content
            result = eval(content)
            if self.verbose:
                print('command:', result['command'], 'content:', result['content'])
            return result  # json格式
        else:
            completion = self.get_response(txt, prompt, **kwargs)
            content = completion.choices[0].message.content
            return content


