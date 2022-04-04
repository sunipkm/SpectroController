# Clock Generator Library
Wrapper around timer library found [here](https://qnaplus.com/implement-periodic-timer-linux/).
Keeping the same front-end, also provides clocks in macOS using Grand Central Dispatch.
Linux version is limited to 1000 clocks, and macOS timer_gen library does not support one-shot mode. Clock functionality are identical across operating systems.

Register your handler function to perform a task at expiration of timer.

To build, execute `make`, and to test, execute `make test`.
