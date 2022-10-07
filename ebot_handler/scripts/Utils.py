from enum import Enum


dropbox = {'Conference-Room': 'DropBox-1',
            'Meeting-Room-drop': 'DropBox-2',
            'Research-Lab': 'DropBox-3'
            }

# enum for task status codes
class TaskStatusCode(Enum):
    SUCCESS = 1
    FAIL = 0
    IN_PROGRESS = 2
    WAITING_FOR_TASK_PUBLISHER = 3

