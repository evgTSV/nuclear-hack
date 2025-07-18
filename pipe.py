class Pipe:
    def __init__(self, value):
        self.value = value

    def __or__(self, func):
        return Pipe(func(self.value))

    def get(self):
        return self.value