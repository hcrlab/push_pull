from collections import namedtuple

# Bin data structure.
#   id: string, 'A' - 'L' according to the diagram on the contest website.
#   visited: boolean, true if the robot has tried to pick an item from this bin
#     at least once, false otherwise.
#   succeeded: boolean, true if the robot successfully picked an item from this
#     bin, false otherwise. succeeded = false either means that there was an
#     unsuccessful attempt, or it hasn't been tried yet, check visited to
#     distinguish between these two cases.
#	attempts_remaining: int, indicates the number of attempts remaining
#	  before this bin should be ignored
BinData = namedtuple('BinData', ['id', 'visited', 'succeeded', 'attempts_remaining'])
