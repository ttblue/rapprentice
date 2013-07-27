"""
Misc functions that are useful in the top-level task-execution scripts
"""


def request_int_in_range(too_high_val):
    while True:
        try:
            choice_ind = int(raw_input())
            if choice_ind <= too_high_val:
                return choice_ind
        except ValueError:            
            pass
        print "Invalid selection. Try again"
