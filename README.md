Markov Decision Process Solver

By Luis Gerardo Huiza Blanco

tested on snappy1.cims.nyu.edu

compile just typing make to use the included Makefile

argument structure: 

./mdp

-max (for maximizing gains, if not present it will minimize gains [optional])

-df (discount factor, if not declared it defaults to 1 [optional])

-tol (tolerance of difference of iterations, defaults to 0.001 [optional])

-iter (max number of iterations to be done on the MDP, defaults to 150 [optional])

[file_name] (name of the file from which to take the information for building the MDP [not_optional])

-v (activates verbose mode and displays the structure of the MDP when building it and the path of each node through each iteration [optional])

output of the code should look something like this:

//the policy of the MDP, if there are any decision nodes it will show their optimal decision; chance, single node, and terminal nodes are not displayed here
A -> Z
B -> Z
C -> B
D -> C
E -> B
F -> E
G -> F

//the reward of each node of the MDP
A=0.797 B=0.996 C=0.967 D=0.767 E=0.994 F=0.986 G=0.942 Y=-1.000 Z=1.000