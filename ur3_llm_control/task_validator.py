"""Pure validator: reject the complete plan before any robot movement."""
import json
from .configuration import OBJECTS, ZONES


class InvalidPlan(ValueError):
    pass


def _unique_keys(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise InvalidPlan('Khóa JSON bị lặp: ' + key)
        value[key] = item
    return value


def validate_plan(raw):
    """Validate basic one-object or advanced multi-object transfer plans.

    Every transfer is deliberately a complete ``pick -> place -> home``
    triplet.  This keeps the robot in a known posture between objects while
    allowing one LLM response to coordinate several objects.
    """
    if isinstance(raw, str):
        if len(raw) > 16000:
            raise InvalidPlan('Phản hồi quá dài')
        try:
            raw = json.loads(raw, object_pairs_hook=_unique_keys)
        except (ValueError, RecursionError) as exc:
            raise InvalidPlan('JSON không hợp lệ: ' + str(exc)) from exc
    if not isinstance(raw, dict):
        raise InvalidPlan('Output phải là JSON object')
    if set(raw) == {'error'} and isinstance(raw['error'], str):
        raise InvalidPlan('LLM từ chối yêu cầu: ' + raw['error'][:300])
    if set(raw) != {'plan'} or not isinstance(raw['plan'], list):
        raise InvalidPlan('Chỉ chấp nhận khóa plan chứa một danh sách')

    plan = raw['plan']
    if len(plan) not in (3, 6, 9):
        raise InvalidPlan('Plan phải gồm 1–3 cụm pick -> place -> home')
    if len(plan) % 3:
        raise InvalidPlan('Plan phải chia hết thành các cụm 3 skill')

    objects, zones = set(), set()
    for offset in range(0, len(plan), 3):
        triplet = plan[offset:offset + 3]
        schemas = [('pick', {'skill', 'object'}),
                   ('place', {'skill', 'object', 'zone'}),
                   ('home', {'skill'})]
        for step, (skill, keys) in zip(triplet, schemas):
            if not isinstance(step, dict) or set(step) != keys or step.get('skill') != skill:
                raise InvalidPlan('Sai skill, thứ tự hoặc tham số: ' + repr(step))
            if 'object' in step and (not isinstance(step['object'], str) or step['object'] not in OBJECTS):
                raise InvalidPlan('INVALID_OBJECT')
            if 'zone' in step and (not isinstance(step['zone'], str) or step['zone'] not in ZONES):
                raise InvalidPlan('INVALID_ZONE')
        obj, dest = triplet[0]['object'], triplet[1]['zone']
        if triplet[1]['object'] != obj:
            raise InvalidPlan('Vật được place phải là vật vừa pick')
        expected = {'zone_a': 'blue_cube', 'zone_b': 'red_cube', 'zone_c': 'yellow_cube'}[dest]
        if obj != expected:
            raise InvalidPlan(
                f'INVALID_OBJECT: {obj} không được đặt vào {dest} theo MSSV 23020734; '
                f'vùng này yêu cầu {expected}')
        if obj in objects:
            raise InvalidPlan('Một vật chỉ được chuyển một lần trong cùng plan')
        if dest in zones:
            raise InvalidPlan('Mỗi vùng chỉ được dùng một lần trong cùng plan')
        objects.add(obj)
        zones.add(dest)
    return [dict(step) for step in plan]


def format_step(step):
    args = [step[k] for k in ('object', 'zone') if k in step]
    return step['skill'] + '(' + ', '.join(args) + ')'
