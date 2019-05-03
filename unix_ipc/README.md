Unix Inter-Process Communication (IPC) Examples
===============================================

## Memory-Mapped Files and Shared Memory

On systems that don't have shared memory (i.e., the Snapdragon Flight Pro board), memory-mapped files provide an alternative that is similar in spirit. With shared memory, processes avoid going through the kernel by allocating a chunk of memory that can be shared between them. Similarly, memory-mapped files avoid kernel / filesystem overhead by mapping file contents into memory. One pro (maybe?) that memory-mapped files have over shared memory is that the memory is also stored on the filesystem after it is used (persistence).

### Timing Stats

| **machine**                             |     |**shmem**|    |     |**mmap**|     |
|-----------------------------------------|-----|-------|------|-----|------|-------|
| **_(latency, in microseconds)_**        |*min*|*mean* |*max* |*min*|*mean*|*max*  |
| Ubuntu 16.04 i7 Desktop                 | 15  | 58    | 333  | 5   | 48   | 609   |
| Snapdragon Flight (Eagle, 8074)         | 38  | 136   | 4878 | 40  | 150  | 2081  |
| Snapdragon Flight Pro (Excelsior, 8096) | -   | -     | -    | 80  | 390  | 11000 |

The fact that on a desktop memory-mapped files are faster than shared memory is odd. See [Evaluation of Inter-Process Communications Mechanisms](http://pages.cs.wisc.edu/~adityav/Evaluation_of_Inter_Process_Communication_Mechanisms.pdf).

#### Resources

- [Beej's Guide to Unix Interprocess Communication](https://beej.us/guide/bgipc/)
- [Interprocess Communication: Memory-mapped files and pipes](https://courses.engr.illinois.edu/cs241/sp2014/lecture/27-IPC.pdf)
- [mmap and resize on new file](https://gist.github.com/marcetcheverry/991042/f8426523406419c0824b519da9bb12fc9713aae6)
- [Synchronization Across Process Boundaries](https://docs.oracle.com/cd/E19455-01/806-5257/6je9h032v/index.html)
- [pthread_cond_wait](https://github.com/angrave/SystemProgramming/wiki/Synchronization,-Part-5:-Condition-Variables#example)