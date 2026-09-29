"""9Router HTTP client. No robot APIs and no rule-based NLP fallback."""
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from .task_validator import validate_plan


class PlannerError(RuntimeError):
    pass


class LLMPlanner:
    def __init__(self, prompt, student):
        self.prompt = prompt + '\nStudent context: ' + json.dumps(student, ensure_ascii=False)
        self.base_url = os.getenv('NINE_ROUTER_BASE_URL', 'http://localhost:20128/v1').rstrip('/')
        self.model = os.getenv('NINE_ROUTER_MODEL', '')
        self.api_key = os.getenv('NINE_ROUTER_API_KEY', '')

    def plan(self, command):
        if not isinstance(command, str) or not command.strip() or len(command) > 2000:
            raise PlannerError('Câu lệnh phải có 1–2000 ký tự')
        if not self.model:
            raise PlannerError('Chưa đặt NINE_ROUTER_MODEL; chọn model đã kết nối trong 9Router')
        payload = {'model': self.model, 'stream': False, 'messages': [
            {'role': 'system', 'content': self.prompt},
            {'role': 'user', 'content': command},
        ]}
        headers = {'Content-Type': 'application/json'}
        if self.api_key:
            headers['Authorization'] = 'Bearer ' + self.api_key
        request = Request(self.base_url + '/chat/completions',
                          data=json.dumps(payload).encode(), headers=headers, method='POST')
        try:
            with urlopen(request, timeout=60) as response:
                body = response.read(1_000_001)
            if len(body) > 1_000_000:
                raise PlannerError('Phản hồi 9Router vượt giới hạn dung lượng')
            data = json.loads(body)
            choice = data['choices'][0]
            if choice.get('finish_reason') not in (None, 'stop'):
                raise PlannerError('LLM chưa kết thúc bình thường: ' + str(choice.get('finish_reason')))
            content = choice['message']['content']
            if not isinstance(content, str):
                raise PlannerError('LLM không trả về nội dung JSON dạng text')
        except HTTPError as exc:
            # Do not leak provider bodies / credentials in terminal or ROS topics.
            raise PlannerError(f'9Router HTTP {exc.code}; kiểm tra model, provider, API key') from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise PlannerError('Không kết nối được 9Router hoặc hết thời gian chờ') from exc
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            raise PlannerError('9Router trả về response không đúng schema chat completions') from exc
        return validate_plan(content)
