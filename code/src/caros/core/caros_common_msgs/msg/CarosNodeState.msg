# state of node (INIT,STOPPED,RUNNING,ERROR,FATALERROR)
string state

bool inError
string error_msg
int64 error_code

# true if
bool changed_event
string previous_state
