"""Small task orchestrators for composed serving demonstrations."""


class CommandServingSequence:
    """Run an arbitrary non-empty payload order with safe tray ownership."""

    TRAY_TASK_NAMES = frozenset({"soda1", "soda2", "cutlery"})

    def __init__(self, named_tasks):
        self._named_tasks = list(named_tasks)
        if not self._named_tasks:
            raise ValueError("serving sequence contains no delivery tasks")
        names = [name for name, _ in self._named_tasks]
        if len(names) != len(set(names)):
            raise ValueError(f"serving sequence contains duplicate tasks: {names}")
        unknown = set(names) - self.TRAY_TASK_NAMES - {"pizza"}
        if unknown:
            raise ValueError(f"serving sequence contains unknown tasks: {sorted(unknown)}")
        self._has_pizza = "pizza" in names
        self._has_tray_payload = any(
            name in self.TRAY_TASK_NAMES for name in names
        )
        self._index = 0
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        for name, task in self._named_tasks:
            if name == "pizza":
                task.set_parallel_tray_deployment(self._has_tray_payload)
            task.initialize(articulation, dof_names)
        order = " -> ".join(name for name, _ in self._named_tasks)
        mode = (
            "parallel-pizza-tray"
            if self._has_pizza and self._has_tray_payload
            else "pizza-only"
            if self._has_pizza
            else "first-payload-deploys-tray"
        )
        print(
            f"[serving-sequence] order={order} mode={mode} (all initialized)",
            flush=True,
        )

    def step(self, articulation):
        if self.done or self.failed:
            return
        name, task = self._named_tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            print(f"[serving-sequence] STOPPED: {name} failed", flush=True)
            return
        if not task.done:
            return
        self._index += 1
        if self._index >= len(self._named_tasks):
            self.done = True
            print("[serving-sequence] all deliveries complete", flush=True)
            return
        next_name, next_task = self._named_tasks[self._index]
        print(
            f"[serving-sequence] {name} complete; starting {next_name}",
            flush=True,
        )
        # Pizza or the first payload task owns deployment. Every later tray
        # payload re-reads its live pose but reuses the already-open trays.
        if hasattr(next_task, "start_with_deployed_trays"):
            next_task.start_with_deployed_trays()

    def close(self):
        for _, task in self._named_tasks:
            try:
                task.close()
            except Exception:
                pass


class PizzaThenSodaTask(CommandServingSequence):
    """Run independent pizza and soda tasks in sequence."""

    def __init__(self, pizza_task, soda_task):
        super().__init__([("pizza", pizza_task), ("soda1", soda_task)])

    def initialize(self, articulation, dof_names):
        super().initialize(articulation, dof_names)

    def step(self, articulation):
        super().step(articulation)

    def close(self):
        super().close()


class PizzaSoda1Soda2Task(CommandServingSequence):
    """Run pizza, soda1, then soda2 in sequence."""

    def __init__(self, pizza_task, soda1_task, soda2_task):
        super().__init__(
            [("pizza", pizza_task), ("soda1", soda1_task), ("soda2", soda2_task)]
        )

    def initialize(self, articulation, dof_names):
        super().initialize(articulation, dof_names)

    def step(self, articulation):
        super().step(articulation)

    def close(self):
        super().close()


class PizzaSoda1Soda2CutleryTask(CommandServingSequence):
    """Run the complete meal delivery sequence in sequence."""

    def __init__(self, pizza_task, soda1_task, soda2_task, cutlery_task):
        super().__init__(
            [
                ("pizza", pizza_task),
                ("soda1", soda1_task),
                ("soda2", soda2_task),
                ("cutlery", cutlery_task),
            ]
        )

    def initialize(self, articulation, dof_names):
        super().initialize(articulation, dof_names)

    def step(self, articulation):
        super().step(articulation)

    def close(self):
        super().close()
