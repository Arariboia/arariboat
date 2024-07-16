import re
import os
import sys

running_from_platformio = True
try:
    Import("env") #type: ignore
except NameError:
    print("Not running from PlatformIO. Standalone mode")
    running_from_platformio = False

def ensure_task_handles(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    
    task_declarations = re.findall(r'CREATE_TASK\((\w+), \s*STACK_SIZE\(\d+\), \s*PRIORITY\(\d+\)\);', content)   
    existing_handles = set(re.findall(r'TaskHandle_t (\w+Handle)', content))

    missing_handles = []
    for task in task_declarations:
        task_handle = f"{task}Handle"
        if task_handle not in existing_handles:
            missing_handles.append(task_handle)

    if missing_handles:
        for handle in missing_handles:
            print(f"Missing handle for task {handle}")
        return False
    return True

current_directory = ""
main_unit_file_path = ""
if running_from_platformio:
    current_directory = env.subst("$PROJECTSRC_DIR") #type: ignore
    main_unit_file_path = os.path.join(current_directory, "main.cpp")
else:
    current_directory = os.path.dirname(os.path.abspath(__file__))
    main_unit_file_path = os.path.join(current_directory, './src/main.cpp')

print(f"Checking task handles in file {main_unit_file_path}")

if not ensure_task_handles(main_unit_file_path):
    print("Task handles check failed")
    sys.exit(1)
print("Task handles check passed")


