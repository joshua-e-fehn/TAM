import enum


class StateType(enum.Enum):
    READY = 'READY'
    GB_TRACK = 'GB_TRACK'
    TRAILING = 'TRAILING'
    OVERTAKE = 'OVERTAKE'
    FTGONLY = 'FTGONLY'
