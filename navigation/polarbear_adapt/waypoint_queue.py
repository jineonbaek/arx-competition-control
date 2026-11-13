"""Ordered, operator-started routes for the single-goal chassis driver."""


class Route:
    def __init__(self):
        self.reset("draft")

    def reset(self, reason):
        self.points = []
        self.index = 0
        self.running = False
        self.state = reason

    def add(self, point):
        if self.running:
            return False
        if self.state != 'draft':
            self.reset('draft')
        self.points.append(point)
        return True

    def start(self):
        if self.running or self.state != 'draft' or not self.points:
            return None
        self.running = True
        self.state = 'running'
        return self.points[self.index]

    def finish(self, outcome):
        if not self.running:
            return None
        if outcome == 'arrived_stopped':
            self.index += 1
            if self.index < len(self.points):
                return self.points[self.index]
            outcome = 'completed'
        self.running = False
        self.state = outcome
        return None
