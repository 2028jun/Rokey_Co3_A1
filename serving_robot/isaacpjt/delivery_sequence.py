"""Small task orchestrators for composed serving demonstrations."""


class PizzaThenSodaTask:
    """Run the independent pizza and soda tasks in a strict sequence."""

    def __init__(self, pizza_task, soda_task):
        self._pizza = pizza_task
        self._soda = soda_task
        self._soda_started = False
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        self._pizza.initialize(articulation, dof_names)
        self._soda.initialize(articulation, dof_names)
        print("[serving-sequence] order=pizza -> soda", flush=True)

    def step(self, articulation):
        if self.done or self.failed:
            return
        if not self._pizza.done:
            self._pizza.step(articulation)
            if self._pizza.failed:
                self.failed = True
                print(
                    "[serving-sequence] STOPPED: pizza task failed; "
                    "soda task will not start",
                    flush=True,
                )
            return
        if not self._soda_started:
            self._soda_started = True
            print(
                "[serving-sequence] pizza complete; starting soda task",
                flush=True,
            )
            self._soda.start_with_deployed_trays()
        self._soda.step(articulation)
        if self._soda.failed:
            self.failed = True
            print("[serving-sequence] STOPPED: soda task failed", flush=True)
        elif self._soda.done:
            self.done = True
            print(
                "[serving-sequence] pizza and soda deliveries complete",
                flush=True,
            )

    def close(self):
        self._pizza.close()
        self._soda.close()


class PizzaSoda1Soda2Task:
    """Run pizza, left-front soda1, then right-front soda2."""

    def __init__(self, pizza_task, soda1_task, soda2_task):
        self._tasks = [pizza_task, soda1_task, soda2_task]
        self._index = 0
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        for task in self._tasks:
            task.initialize(articulation, dof_names)
        print(
            "[serving-sequence] order=pizza -> soda1 -> soda2",
            flush=True,
        )

    def step(self, articulation):
        if self.done or self.failed:
            return
        task = self._tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            names = ("pizza", "soda1", "soda2")
            print(
                f"[serving-sequence] STOPPED: {names[self._index]} failed",
                flush=True,
            )
            return
        if not task.done:
            return
        self._index += 1
        if self._index >= len(self._tasks):
            self.done = True
            print(
                "[serving-sequence] pizza, soda1 and soda2 deliveries complete",
                flush=True,
            )
            return
        names = ("pizza", "soda1", "soda2")
        print(
            f"[serving-sequence] {names[self._index - 1]} complete; "
            f"starting {names[self._index]}",
            flush=True,
        )
        self._tasks[self._index].start_with_deployed_trays()

    def close(self):
        for task in self._tasks:
            task.close()


class PizzaSoda1Soda2CutleryTask:
    """Run the complete meal delivery sequence including the cutlery box."""

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
            "[serving-sequence] order=pizza -> soda1 -> soda2 -> cutlery",
            flush=True,
        )

    def step(self, articulation):
        if self.done or self.failed:
            return
        task = self._tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            print(
                f"[serving-sequence] STOPPED: "
                f"{self._names[self._index]} failed",
                flush=True,
            )
            return
        if not task.done:
            return
        completed_name = self._names[self._index]
        self._index += 1
        if self._index >= len(self._tasks):
            self.done = True
            print(
                "[serving-sequence] pizza, soda1, soda2 and cutlery "
                "deliveries complete",
                flush=True,
            )
            return
        print(
            f"[serving-sequence] {completed_name} complete; "
            f"starting {self._names[self._index]}",
            flush=True,
        )
        self._tasks[self._index].start_with_deployed_trays()

    def close(self):
        for task in self._tasks:
            task.close()
