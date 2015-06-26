#include <stdio.h>

#include "djk.h"


int main()
{
	std::vector< std::vector< float > > edges;

	int size = 5;
	for (int i=0;i<size; i++)
	{
		std::vector<float> eds;
		for (int j=0;j<size; j++)
		{
			eds.push_back(DJ_INFINITY);
		}
		edges.push_back(eds);
	}

	edges[0][1] = 1;
	edges[1][0] = 1;

	edges[1][2] = 3;
	edges[2][1] = 3;

	edges[1][3] = 1;
	edges[3][1] = 1;

	edges[3][2] = 1;
	edges[2][3] = 1;

	Dijkstra d = Dijkstra(&edges);

	d.calculateDistance(0);

	for (int goal=0; goal<size; goal++)
	{
		std::vector<int> path;
		float ret = d.go(goal, path);
		printf("RET: %f\n", ret);
		for (int i=0; i<path.size(); i++)
		{
			printf("%d", path[i]);
		}
		printf("\n");
	}

}