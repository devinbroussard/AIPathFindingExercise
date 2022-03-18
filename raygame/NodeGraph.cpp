#include "NodeGraph.h"
#include <raylib.h>
#include <xlocinfo>

DynamicArray<NodeGraph::Node*> reconstructPath(NodeGraph::Node* start, NodeGraph::Node* end)
{
	DynamicArray<NodeGraph::Node*> path;
	NodeGraph::Node* currentNode = end;

	while (currentNode != start->previous)
	{
		currentNode->color = 0xFFFF00;
		path.insert(currentNode, 0);
		currentNode = currentNode->previous;
	}

	return path;
}

float diagonalDistance(NodeGraph::Node* node, NodeGraph::Node* goal, float cardinalCost, float diagonalCost)
{
	float displacementX = abs(node->position.x - goal->position.x);
	float displacementY = abs(node->position.y - goal->position.y);

	return cardinalCost * (displacementX + displacementY) + (diagonalCost - 2 * cardinalCost) * fmin(displacementX, displacementY);
}

void sortFScore(DynamicArray<NodeGraph::Node*>& nodes)
{
	NodeGraph::Node* key = nullptr;
	int j = 0;

	for (int i = 1; i < nodes.getLength(); i++) {
		key = nodes[i];
		j = i - 1;
		while (j >= 0 && nodes[j]->fScore > key->fScore) {
			nodes[j + 1] = nodes[j];
			j--;
		}

		nodes[j + 1] = key;
	}
}

//Applies Dijkstra's algorithm to find the shortest path from one node to another
DynamicArray<NodeGraph::Node*> NodeGraph::findPath(Node* start, Node* goal) {
	NodeGraph::Node* currentNode; //The node that is currently being processed
	float gScore = 0;
	//A list that holds nodes currently being processed
	DynamicArray<NodeGraph::Node*> openList = DynamicArray<NodeGraph::Node*>();
	//A list that holds nodes that have already been processed
	DynamicArray<NodeGraph::Node*> closedList = DynamicArray<NodeGraph::Node*>();

	openList.addItem(start); //Adds the starting node to the openList array
	currentNode = start; //Sets the current node to be be the given starting node

	//Searches for a path until the isOpenListEmpty bool is set to true
	while (openList.getLength() > 0) {
		NodeGraph::Node* shortestGScore = currentNode->edges[0].target;
		for (int i = 0; i < currentNode->edges.getLength(); i++) {
			//Adds the node to the open list if it is not already in it
			if (!openList.contains(currentNode->edges[i].target))
				openList.addItem(currentNode->edges[i].target);

			//Adds the current distance to the edge's cost, and add the node to the open list
			float nodeGScore = currentNode->edges[i].cost + gScore;
			if (currentNode->edges[i].target->gScore > nodeGScore)
				currentNode->edges[i].target->gScore = nodeGScore;

			currentNode->edges[i].target->previous = currentNode;
			
			if (currentNode->edges[i].target->gScore < shortestGScore->gScore)
				shortestGScore = currentNode->edges[i].target;
		}

		currentNode = shortestGScore;
		//Removes the current node from the open list array, since we are processing it now
		openList.remove(currentNode);
		//Adds the current node to the closed list so that we don't process it again
		closedList.addItem(currentNode);
		
	}

	return reconstructPath(start, goal);
}

void NodeGraph::drawGraph(Node* start)
{
	DynamicArray<Node*> drawnList = DynamicArray<Node*>();
	drawConnectedNodes(start, drawnList);
}

void NodeGraph::drawNode(Node* node, int color)
{
	static char buffer[10];
	sprintf_s(buffer, "%.0f", node->gScore);

	//Draw the circle for the outline
	DrawCircle((int)node->position.x, (int)node->position.y, 16, GetColor(color));
	//Draw the inner circle
	DrawCircle((int)node->position.x, (int)node->position.y, 14, BLACK);
	//Draw the text
	DrawText(buffer, (int)node->position.x, (int)node->position.y, 24, RAYWHITE);
}

void NodeGraph::drawConnectedNodes(Node* node, DynamicArray<Node*>& drawnList)
{
	drawnList.addItem(node);

	for (int i = 0; i < node->edges.getLength(); i++)
	{
		Edge e = node->edges[i];
		//Draw the Edge
		DrawLine((int)node->position.x, (int)node->position.y, (int)e.target->position.x, (int)e.target->position.y, WHITE);
		//Draw the cost
		MathLibrary::Vector2 costPos = { (node->position.x + e.target->position.x) / 2, (node->position.y + e.target->position.y) / 2 };
		static char buffer[10];
		sprintf_s(buffer, "%.0f", e.cost);
		DrawText(buffer, (int)costPos.x, (int)costPos.y, 16, RAYWHITE);
		//Draw the target node
		if (!drawnList.contains(e.target)) {
			drawConnectedNodes(e.target, drawnList);
		}
	}
}
