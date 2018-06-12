bool success
string message  # informational, e.g. for error messages
int32 err_no    # 0 Normal end, -1 Error
uint16 job_line # Job line
uint16 step     # Step
string job_name	# Job name (up to 32 characters for a job name)
int16 task_no	# Task number 0: Master task, 1-15: Subtask 1 - 15
string master_job_name