"""Sequential, fail-stop execution for basic and advanced plans."""
from .task_validator import validate_plan, format_step


class SkillExecutor:
    def __init__(self, skills):
        self.skills = skills
        self.faulted = False

    def reset(self):
        """Unlock execution after a successful physical scene reset."""
        self.faulted = False

    def execute(self, plan, log=print):
        steps = validate_plan({'plan': plan})
        if self.faulted:
            log('TASK FAILED: executor đã lỗi; nhấn h để reset cảnh rồi chạy lại')
            return False
        log('EXECUTION:')
        for offset in range(0, len(steps), 3):
            pick_step, place_step, home_step = steps[offset:offset + 3]
            # In plus mode the extra orange cube may occupy the destination.
            # Clear it immediately before the first normal object that needs
            # that zone, then continue with the original validated triplet.
            if (getattr(self.skills, 'plus_active', False)
                    and getattr(self.skills, 'plus_zone', None) == place_step['zone']):
                log(f'PLUS: orange_cube đang ở {place_step["zone"]}; chuyển sang temporary_zone')
                status = self.skills.clear_plus_zone()
                for detail in ('pick(orange_cube)', 'place(orange_cube, temporary_zone)', 'home()'):
                    log(f'{detail:40s} {status}')
                if status != 'SUCCESS':
                    self.faulted = True
                    log('TASK FAILED — không thể giải phóng zone plus; nhấn h để reset cảnh')
                    return False
            status = self.skills.preflight(pick_step['object'], place_step['zone'])
            if status != 'SUCCESS':
                log('TASK FAILED: ' + status)
                self.faulted = True
                return False
            for step in (pick_step, place_step, home_step):
                try:
                    if step['skill'] == 'pick':
                        status = self.skills.pick(step['object'])
                    elif step['skill'] == 'place':
                        status = self.skills.place(step['object'], step['zone'])
                    else:
                        status = self.skills.home()
                except Exception as exc:
                    status = 'FAILED: ' + str(exc)
                log(f'{format_step(step):40s} {status}')
                if status != 'SUCCESS':
                    self.faulted = True
                    log('TASK FAILED — dừng tại skill bị lỗi; nhấn h để reset cảnh')
                    return False
        log('TASK SUCCESS')
        return True
