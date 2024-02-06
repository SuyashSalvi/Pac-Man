AI - Algos:

1: Breadth-First Search
In the breadthFirstSearch function in search.py, the breadth-first graph search algorithm (states/locations that are already visited will not be visited again) is implemented, for Pacman to find the path to the dot in the maze. 
Running the code:
python pacman.py -l tinyMaze -p SearchAgent -a fn=breadthFirstSearch
python pacman.py -l smallMaze -p SearchAgent -a fn=breadthFirstSearch
python pacman.py -l mediumMaze -p SearchAgent -a fn=breadthFirstSearch -z .5 --frameTime 0
python pacman.py -l bigMaze -p SearchAgent -a fn=breadthFirstSearch -z .5 --frameTime 0

2: A* Search
The A* graph search implemented in function aStarSearch in search.py, using heuristic function, for Pacman to find the path to the dot in the maze. 
Heuristics take two arguments: a state in the search problem (the main argument), and the problem itself (for reference information).
Running the code:
python pacman.py -l tinyMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
python pacman.py -l smallMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
python pacman.py -l mediumMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic --frameTime 0
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic --frameTime 0
Notes:
• If Pacman moves too slowly for you, try the option --frameTime 0.
• All of the search functions return a list of actions that will lead the agent from the start to the goal.
