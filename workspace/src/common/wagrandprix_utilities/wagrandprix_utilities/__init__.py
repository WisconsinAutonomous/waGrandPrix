def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def scale_to_range(value, old_min, old_max, new_min, new_max):
    return (((value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min