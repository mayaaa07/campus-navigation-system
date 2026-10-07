#include <stdio.h>

#define MAX 20
#define INF 99999

int graph[MAX][MAX];
char locations[MAX][30];
int n;

void printPath(int parent[], int v)
{
    if (parent[v] == -1)
        return;

    printPath(parent, parent[v]);
    printf(" -> %s", locations[v]);
}

void dijkstra(int source)
{
    int dist[MAX];
    int visited[MAX];
    int parent[MAX];

    for (int i = 0; i < n; i++)
    {
        dist[i] = INF;
        visited[i] = 0;
        parent[i] = -1;
    }

    dist[source] = 0;

    for (int count = 0; count < n - 1; count++)
    {
        int min = INF;
        int u = -1;

        for (int i = 0; i < n; i++)
        {
            if (!visited[i] && dist[i] < min)
            {
                min = dist[i];
                u = i;
            }
        }

        if (u == -1)
            break;

        visited[u] = 1;

        for (int v = 0; v < n; v++)
        {
            if (!visited[v] &&
                graph[u][v] != INF &&
                dist[u] + graph[u][v] < dist[v])
            {
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }
        }
    }

    printf("\n=====================================\n");
    printf("Shortest Paths from %s\n", locations[source]);
    printf("=====================================\n");

    for (int i = 0; i < n; i++)
    {
        if (i == source)
            continue;

        printf("\nDestination : %s\n", locations[i]);

        if (dist[i] == INF)
        {
            printf("Distance    : INF\n");
            printf("Path        : No Path Available\n");
        }
        else
        {
            printf("Distance    : %d\n", dist[i]);
            printf("Path        : %s", locations[source]);
            printPath(parent, i);
            printf("\n");
        }
    }

    printf("\n\nDistance Table\n");
    printf("-------------------------------\n");
    printf("Location\tDistance\n");
    printf("-------------------------------\n");

    for (int i = 0; i < n; i++)
    {
        if (dist[i] == INF)
            printf("%s\t\tINF\n", locations[i]);
        else
            printf("%s\t\t%d\n", locations[i], dist[i]);
    }
}

void displayMatrix()
{
    printf("\nAdjacency Matrix\n\n");

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (graph[i][j] == INF)
                printf("INF\t");
            else
                printf("%d\t", graph[i][j]);
        }
        printf("\n");
    }
}

void createGraph()
{
    int roads;

    printf("\nEnter Number of Locations: ");
    scanf("%d", &n);

    for (int i = 0; i < n; i++)
    {
        printf("Enter Location %d Name: ", i + 1);
        scanf("%s", locations[i]);
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
                graph[i][j] = 0;
            else
                graph[i][j] = INF;
        }
    }

    printf("\nEnter Number of Roads: ");
    scanf("%d", &roads);

    printf("\nEnter Road Details (Source Destination Distance)\n");

    for (int i = 0; i < roads; i++)
    {
        int u, v, w;

        printf("Road %d : ", i + 1);
        scanf("%d %d %d", &u, &v, &w);

        graph[u - 1][v - 1] = w;
        graph[v - 1][u - 1] = w;
    }

    printf("\nCampus Graph Created Successfully!\n");
}

int main()
{
    int choice;
    int source;

    do
    {
        printf("\n=================================\n");
        printf(" CAMPUS SHORTEST ROUTE FINDER\n");
        printf("=================================\n");
        printf("1. Create Campus Graph\n");
        printf("2. Display Adjacency Matrix\n");
        printf("3. Find Shortest Routes\n");
        printf("4. Exit\n");
        printf("Enter Choice : ");
        scanf("%d", &choice);

        switch (choice)
        {
        case 1:
            createGraph();
            break;

        case 2:
            displayMatrix();
            break;

        case 3:
            printf("\nLocations:\n");

            for (int i = 0; i < n; i++)
            {
                printf("%d. %s\n", i + 1, locations[i]);
            }

            printf("\nSelect Source Location: ");
            scanf("%d", &source);

            dijkstra(source - 1);
            break;

        case 4:
            printf("\nExiting Program...\n");
            break;

        default:
            printf("\nInvalid Choice!\n");
        }

    } while (choice != 4);

    return 0;
}
