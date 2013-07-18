import heapq
class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.

      Note that this PriorityQueue does not allow you to change the priority
      of an item.  However, you may insert the same item multiple times with
      different priorities.
    """
    def  __init__(self):
        self.heap = []

    def push(self, item, priority, secondary):
        pair = (priority,secondary,item)
        heapq.heappush(self.heap,pair)

    def pop(self):
        (priority,secondary,item) = heapq.heappop(self.heap)
        return (priority,item)
    
    def isEmpty(self):
        return len(self.heap) == 0


queue = PriorityQueue()
queue.push((1,1), 1, '2nd')
queue.push((2,2), 2, '2')

for i in queue.heap:
    print i

queue.pop()

for i in queue.heap:
    print i
