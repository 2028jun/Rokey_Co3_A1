"""Small task orchestrators for composed serving demonstrations."""


class PizzaThenSodaTask:
    """Run independent pizza and soda tasks in sequence."""

    def __init__(self, pizza_task, soda_task):
        self._pizza = pizza_task
        self._soda = soda_task
        self._soda_started = False
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        self._pizza.initialize(articulation, dof_names)
        self._soda.initialize(articulation, dof_names)
        print("[serving-sequence] order=pizza -> soda (all initialized)", flush=True)

    def step(self, articulation):
        if self.done or self.failed:
            return
        if not self._pizza.done:
            self._pizza.step(articulation)
            if self._pizza.failed:
                self.failed = True
                print("[serving-sequence] STOPPED: pizza task failed", flush=True)
            return
        if not self._soda_started:
            self._soda_started = True
            print("[serving-sequence] pizza complete; starting soda task", flush=True)
            self._soda.start_with_deployed_trays()
        self._soda.step(articulation)
        if self._soda.failed:
            self.failed = True
            print("[serving-sequence] STOPPED: soda task failed", flush=True)
        elif self._soda.done:
            self.done = True
            print("[serving-sequence] pizza and soda deliveries complete", flush=True)

    def close(self):
        for task in (self._pizza, self._soda):
            try:
                task.close()
            except Exception:
                pass


class PizzaSoda1Soda2Task:
    """Run pizza, soda1, then soda2 in sequence."""

    def __init__(self, pizza_task, soda1_task, soda2_task):
        self._tasks = [pizza_task, soda1_task, soda2_task]
        self._names = ("pizza", "soda1", "soda2")
        self._index = 0
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        for task in self._tasks:
            task.initialize(articulation, dof_names)
        print("[serving-sequence] order=pizza -> soda1 -> soda2 (all initialized)", flush=True)

    def step(self, articulation):
        if self.done or self.failed:
            return
        task = self._tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            print(f"[serving-sequence] STOPPED: {self._names[self._index]} failed", flush=True)
            return
        if not task.done:
            return
        completed_name = self._names[self._index]
        self._index += 1
        if self._index >= len(self._tasks):
            self.done = True
            print("[serving-sequence] pizza, soda1 and soda2 deliveries complete", flush=True)
            return
        print(f"[serving-sequence] {completed_name} complete; starting {self._names[self._index]}", flush=True)
        next_task = self._tasks[self._index]
        if hasattr(next_task, "start_with_deployed_trays"):
            next_task.start_with_deployed_trays()

    def close(self):
        for task in self._tasks:
            try:
                task.close()
            except Exception:
                pass


class PizzaSoda1Soda2CutleryTask:
    """Run the complete meal delivery sequence in sequence."""

    def __init__(self, pizza_task, soda1_task, soda2_task, cutlery_task):
        self._tasks = [pizza_task, soda1_task, soda2_task, cutlery_task]
        self._names = ("pizza", "soda1", "soda2", "cutlery")
        self._index = 0
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        for task in self._tasks:
            task.initialize(articulation, dof_names)
        print(
            "[serving-sequence] order=pizza -> soda1 -> soda2 -> cutlery (all initialized)",
            flush=True,
        )

    def step(self, articulation):
        if self.done or self.failed:
            return
        task = self._tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            print(f"[serving-sequence] STOPPED: {self._names[self._index]} failed", flush=True)
            return
        if not task.done:
            return
        completed_name = self._names[self._index]
        self._index += 1
        if self._index >= len(self._tasks):
            self.done = True
            print("[serving-sequence] pizza, soda1, soda2 and cutlery deliveries complete", flush=True)
            return
        print(f"[serving-sequence] {completed_name} complete; starting {self._names[self._index]}", flush=True)
        next_task = self._tasks[self._index]
        if hasattr(next_task, "start_with_deployed_trays"):
            next_task.start_with_deployed_trays()

    def close(self):
        for task in self._tasks:
            try:
                task.close()
            except Exception:
                pass
