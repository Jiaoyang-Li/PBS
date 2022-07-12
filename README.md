# PBS
A suboptimal solver for Multi-Agent Path Finding

Priority-Based Search (PBS) is an efficient suboptimal algorithm for solving Multi-Agent Path Finding (MAPF).
More details can be found in our paper at AAAI 2019 [1]. 
(This implementation is not the original code for producing the results in the paper.)

The implementation provides a SIPP option that uses SIPPS [2] (instead of state-time A*) 
in the low level of PBS to plan paths for agents.

The code requires the external library BOOST (https://www.boost.org/). 
After you installed BOOST and downloaded the source code, 
go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

You also need to download the MAPF instances from the MAPF benchmark (https://movingai.com/benchmarks/mapf/index.html).

Then, you are able to run the code:
```
./pbs -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 50 -t 60
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file that contains the search statistics
- outputPaths: the output file that contains the paths 
- k: the number of agents
- t: the runtime limit

You can find more details and explanations for all parameters with:
```
./pbs --help
```

## License
PBS is released under USC – Research License. See license.md for further details.
 
## References
[1] Hang Ma, Daniel Harabor, Peter J. Stuckey, Jiaoyahng Li and S. Koenig. 
Searching with Consistent Prioritization for Multi-Agent Path Finding. 
In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), 7643-7650, 2019.
[2] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey and Sven Koenig.
MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search
In Proceedings of the AAAI Conference on Artificial Intelligence, (in print), 2022.
