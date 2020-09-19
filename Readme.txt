Tested: Compiled in Visual Studio 2015

Libraries Required: 
boost
Google Sparsehash (included)

Command:
<Algorithm>.exe -m MapFile.map -a AgentFile.agents -o Output.csv -s CBSH -k <NumOfAgents> -p <PriorChoice>

CBS:
<Algorithm> = CBSH
<PriorChoice> = 0

CBSw/P:
<Algorithm> = CBSH
<PriorChoice> = 2

PBS:
<Algorithm> = PRIORITY
<PriorChoice> = 2

FIX:
<Algorithm> = PRIORITY
<PriorChoice> = 3

LH:
<Algorithm> = FIX
<PriorChoice> = 1

sH:
<Algorithm> = FIX
<PriorChoice> = 2

RND:
<Algorithm> = FIX
<PriorChoice> = 0

Well-formed version of <Algorithm>:
<Algorithm>-G