/*
 * Smart Campus Navigation System using Dijkstra's Algorithm
 * ---------------------------------------------------------
 * Stores the campus as a weighted undirected graph (adjacency matrix, road
 * lengths in metres) and computes the shortest route from any chosen
 * starting point to all other buildings.
 *
 * Time  : O(V^2)  (array-based minimum selection)
 * Space : O(V^2)  (matrix) + O(V) (working arrays)
 */
#include <stdio.h>
#include <string.h>

#define MAX_SPOTS   12
#define LABEL_SIZE  32
#define NO_ROAD     1000000
#define NO_PARENT   (-1)

typedef struct {
    int  spots;                                /* number of buildings        */
    char label[MAX_SPOTS][LABEL_SIZE];         /* building names             */
    int  road[MAX_SPOTS][MAX_SPOTS];           /* road length in metres      */
} CampusMap;

typedef struct {
    int start;                                 /* chosen starting building   */
    int best[MAX_SPOTS];                       /* shortest known distance    */
    int prev[MAX_SPOTS];                       /* previous building on route */
    int ready;                                 /* 1 after a successful run   */
} RouteTable;

static CampusMap campus;
static RouteTable routes = { -1, {0}, {0}, 0 };

/* ---------- small input helpers ---------- */
static void flushLine(void) {
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF) { }
}

static int askNumber(const char *prompt, int low, int high) {
    int value;
    printf("%s", prompt);
    if (scanf("%d", &value) != 1) { flushLine(); return -1; }
    if (value < low || value > high) return -1;
    return value;
}

static int mapLoaded(void) {
    if (campus.spots == 0) {
        printf("\n  >> No campus map available. Use option 1 or 2 first.\n");
        return 0;
    }
    return 1;
}

static int runDone(void) {
    if (!routes.ready) {
        printf("\n  >> Routes not calculated. Use option 4 first.\n");
        return 0;
    }
    return 1;
}

static void clearMap(int count) {
    int a, b;
    campus.spots = count;
    for (a = 0; a < count; a++)
        for (b = 0; b < count; b++)
            campus.road[a][b] = (a == b) ? 0 : NO_ROAD;
    routes.ready = 0;
    routes.start = -1;
}

static void addRoad(int a, int b, int metres) {
    campus.road[a][b] = metres;
    campus.road[b][a] = metres;
}

/* ---------- map creation ---------- */
static void loadDemoMap(void) {
    static const char *names[6] = {
        "Parking_Area", "Innovation_Hub", "Examination_Cell",
        "Seminar_Hall", "Sports_Complex", "Research_Center"
    };
    int i;
    clearMap(6);
    for (i = 0; i < 6; i++) strcpy(campus.label[i], names[i]);

    addRoad(0, 1, 120);   /* Parking      - Innovation  */
    addRoad(0, 2, 200);   /* Parking      - Examination */
    addRoad(1, 2,  70);   /* Innovation   - Examination */
    addRoad(1, 3, 150);   /* Innovation   - Seminar     */
    addRoad(2, 3,  90);   /* Examination  - Seminar     */
    addRoad(2, 4, 240);   /* Examination  - Sports      */
    addRoad(3, 5, 110);   /* Seminar      - Research    */
    addRoad(4, 5,  60);   /* Sports       - Research    */
    printf("\n  Demo campus loaded: 6 buildings, 8 roads.\n");
}

static void buildCustomMap(void) {
    int count, roads, i, a, b, len;

    count = askNumber("\n  How many buildings (2-12)? ", 2, MAX_SPOTS);
    if (count < 0) { printf("  >> Invalid count.\n"); return; }
    clearMap(count);

    printf("  Name the buildings (single word, use '_' for gaps):\n");
    for (i = 0; i < count; i++) {
        printf("    Building %d: ", i + 1);
        scanf("%31s", campus.label[i]);
    }

    roads = askNumber("  How many roads? ", 0, count * (count - 1) / 2);
    if (roads < 0) { printf("  >> Invalid road count.\n"); campus.spots = 0; return; }

    printf("  Enter each road as: building_no building_no length_in_metres\n");
    for (i = 0; i < roads; i++) {
        printf("    Road %d: ", i + 1);
        if (scanf("%d %d %d", &a, &b, &len) != 3) {
            flushLine();
            printf("    >> Numbers only. Try again.\n");
            i--; continue;
        }
        if (a < 1 || a > count || b < 1 || b > count || a == b) {
            printf("    >> Bad building numbers. Try again.\n");
            i--; continue;
        }
        if (len < 0) {
            printf("    >> Length cannot be negative. Try again.\n");
            i--; continue;
        }
        addRoad(a - 1, b - 1, len);
    }
    printf("\n  Campus map saved.\n");
}

/* ---------- display ---------- */
static void showRoadTable(void) {
    int a, b;
    if (!mapLoaded()) return;

    printf("\n  Road length table in metres ('--' = no direct road)\n\n      ");
    for (b = 0; b < campus.spots; b++) printf("%6d", b + 1);
    printf("\n");
    for (a = 0; a < campus.spots; a++) {
        printf("  %2d  ", a + 1);
        for (b = 0; b < campus.spots; b++) {
            if (campus.road[a][b] == NO_ROAD) printf("%6s", "--");
            else printf("%6d", campus.road[a][b]);
        }
        printf("   %s\n", campus.label[a]);
    }
}

static void listBuildings(void) {
    int i;
    printf("\n  Buildings:\n");
    for (i = 0; i < campus.spots; i++)
        printf("    %2d. %s\n", i + 1, campus.label[i]);
}

/* ---------- Dijkstra core ---------- */
static void computeRoutes(int start) {
    int settled[MAX_SPOTS] = {0};
    int i, round, cur, nxt, smallest;

    for (i = 0; i < campus.spots; i++) {
        routes.best[i] = NO_ROAD;
        routes.prev[i] = NO_PARENT;
    }
    routes.best[start] = 0;
    routes.start = start;

    for (round = 0; round < campus.spots; round++) {
        /* choose the unsettled building that is currently nearest */
        cur = -1;
        smallest = NO_ROAD;
        for (i = 0; i < campus.spots; i++)
            if (!settled[i] && routes.best[i] < smallest) {
                smallest = routes.best[i];
                cur = i;
            }
        if (cur == -1) break;             /* everything left is unreachable */
        settled[cur] = 1;

        /* try to improve each neighbour by going through 'cur' */
        for (nxt = 0; nxt < campus.spots; nxt++) {
            if (settled[nxt] || campus.road[cur][nxt] == NO_ROAD) continue;
            if (routes.best[cur] + campus.road[cur][nxt] < routes.best[nxt]) {
                routes.best[nxt] = routes.best[cur] + campus.road[cur][nxt];
                routes.prev[nxt] = cur;
            }
        }
    }
    routes.ready = 1;
}

static void printRoute(int target) {
    if (routes.prev[target] != NO_PARENT) {
        printRoute(routes.prev[target]);
        printf(" -> ");
    }
    printf("%s", campus.label[target]);
}

/* ---------- menu actions ---------- */
static void actionPickAndRun(void) {
    int choice;
    int i;
    if (!mapLoaded()) return;
    listBuildings();
    choice = askNumber("\n  Starting building number: ", 1, campus.spots);
    if (choice < 0) { printf("  >> Invalid building number.\n"); return; }

    computeRoutes(choice - 1);
    printf("\n  Routes calculated from %s.\n", campus.label[routes.start]);
    printf("\n  Working table after the run:\n");
    printf("  %-3s %-20s %-10s %s\n", "No", "Building", "Best(m)", "Came from");
    for (i = 0; i < campus.spots; i++) {
        printf("  %-3d %-20s ", i + 1, campus.label[i]);
        if (routes.best[i] == NO_ROAD) printf("%-10s ", "unreachable");
        else printf("%-10d ", routes.best[i]);
        if (routes.prev[i] == NO_PARENT) printf("-\n");
        else printf("%s\n", campus.label[routes.prev[i]]);
    }
}

static void actionSingleDestination(void) {
    int target;
    if (!mapLoaded() || !runDone()) return;
    listBuildings();
    target = askNumber("\n  Destination building number: ", 1, campus.spots);
    if (target < 0) { printf("  >> Invalid building number.\n"); return; }
    target--;

    printf("\n  From %s to %s\n", campus.label[routes.start], campus.label[target]);
    if (routes.best[target] == NO_ROAD) {
        printf("  No route exists.\n");
        return;
    }
    printf("  Distance : %d m\n  Route    : ", routes.best[target]);
    printRoute(target);
    printf("\n");
}

static void actionFullReport(void) {
    int i;
    if (!mapLoaded() || !runDone()) return;

    printf("\n  Navigation report from %s\n\n", campus.label[routes.start]);
    printf("  %-20s %-9s %s\n", "Destination", "Metres", "Route");
    printf("  -------------------------------------------------------------------------\n");
    for (i = 0; i < campus.spots; i++) {
        if (i == routes.start) continue;
        printf("  %-20s ", campus.label[i]);
        if (routes.best[i] == NO_ROAD) {
            printf("%-9s %s\n", "--", "not reachable");
        } else {
            printf("%-9d ", routes.best[i]);
            printRoute(i);
            printf("\n");
        }
    }
}

static void showComplexityNote(void) {
    printf("\n  Time  : O(V^2) - V rounds, each doing an O(V) minimum search\n");
    printf("          and an O(V) neighbour scan of one matrix row.\n");
    printf("  Space : O(V^2) for the road matrix, O(V) for the working arrays.\n");
}

int main(void) {
    int option;

    printf("==========================================================\n");
    printf("   SMART CAMPUS NAVIGATION SYSTEM  (Dijkstra's Algorithm)\n");
    printf("==========================================================\n");

    do {
        printf("\n  [1] Load demo campus map\n");
        printf("  [2] Create my own campus map\n");
        printf("  [3] Show road length table\n");
        printf("  [4] Choose start point and calculate routes\n");
        printf("  [5] Route to one destination\n");
        printf("  [6] Full navigation report\n");
        printf("  [0] Quit\n");
        option = askNumber("  Select: ", 0, 6);

        switch (option) {
            case 1: loadDemoMap();          break;
            case 2: buildCustomMap();       break;
            case 3: showRoadTable();        break;
            case 4: actionPickAndRun();     break;
            case 5: actionSingleDestination(); break;
            case 6: actionFullReport();     break;
            case 0: showComplexityNote();
                    printf("\n  Session closed.\n"); break;
            default: printf("\n  >> Please select a number between 0 and 6.\n");
        }
    } while (option != 0);

    return 0;
}
