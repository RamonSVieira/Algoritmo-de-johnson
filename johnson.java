import java.util.Arrays;

public class GFG {

    // Define infinito como um valor inteiro muito grande
    private static final int INF = Integer.MAX_VALUE;

    // Função para encontrar o vértice com a menor distância
    // a partir da origem que ainda não foi incluído na árvore de caminhos mais curtos
    private static int minDistance(int[] dist, boolean[] sptSet) {
        int min = INF, minIndex = 0;
        for (int v = 0; v < dist.length; v++) {
            // Atualiza minIndex se uma distância menor for encontrada
            if (!sptSet[v] && dist[v] <= min) {
                min = dist[v];
                minIndex = v;
            }
        }
        return minIndex;
    }

    // Função para executar o algoritmo de Dijkstra no grafo modificado
    private static void dijkstraAlgorithm(int[][] graph, int[][] alteredGraph, int source) {
        int V = graph.length;  // Número de vértices
        int[] dist = new int[V];  // Array de distâncias para armazenar a menor distância a partir da origem
        boolean[] sptSet = new boolean[V];  // Array booleano para acompanhar os vértices visitados

        // Inicializa as distâncias com infinito e a distância da origem como 0
        Arrays.fill(dist, INF);
        dist[source] = 0;

        // Calcula o caminho mais curto para todos os vértices
        for (int count = 0; count < V - 1; count++) {
            // Seleciona o vértice com a menor distância que ainda não foi visitado
            int u = minDistance(dist, sptSet);
            sptSet[u] = true;  // Marca esse vértice como visitado

            // Atualiza os valores de distância para os vértices adjacentes
            for (int v = 0; v < V; v++) {
                // Verifica se há uma atualização para o valor da distância
                if (!sptSet[v] && graph[u][v] != 0 && dist[u] != INF && dist[u] + alteredGraph[u][v] < dist[v]) {
                    dist[v] = dist[u] + alteredGraph[u][v];
                }
            }
        }

        // Imprime as menores distâncias a partir do vértice de origem
        System.out.println("Menor distância a partir do vértice " + source + ":");
        for (int i = 0; i < V; i++) {
            System.out.println("Vértice " + i + ": " + (dist[i] == INF ? "INF" : dist[i]));
        }
    }

    // Função para executar o algoritmo de Bellman-Ford para calcular as menores distâncias
    // de um vértice de origem para todos os outros vértices
    private static int[] bellmanFordAlgorithm(int[][] edges, int V) {
        // Array de distâncias com um vértice extra (virtual)
        int[] dist = new int[V + 1];
        // preenche todo o array infinito
        Arrays.fill(dist, INF);
        dist[V] = 0;  // A distância para o novo vértice de origem (vértice adicionado) é 0

        // Adiciona arestas do novo vértice de origem para todos os vértices originais
        int[][] edgesWithExtra = Arrays.copyOf(edges, edges.length + V);
        for (int i = 0; i < V; i++) {
            edgesWithExtra[edges.length + i] = new int[]{V, i, 0};
        }

        // Relaxa todas as arestas |V| - 1 vezes
        for (int i = 0; i < V; i++) {
            for (int[] edge : edgesWithExtra) {
                // edge[0] representa a origem da aresta
                // edge[1] representa o vértice de destino da aresta
                // edge[2] representa o peso da aresta(custo de A para B)

                // dist[edge[0]] != INF Certifica que a distancia é conhecida
                // Verifica se o caminho passando pela aresta edge (da origem edge[0] 
                // até o destino edge[1]) resulta em uma distância menor do que a distância 
                // atualmente conhecida para edge[1]
                if (dist[edge[0]] != INF && dist[edge[0]] + edge[2] < dist[edge[1]]) {
                    // Encontramos um caminho mais curto, então atualizamos.
                    dist[edge[1]] = dist[edge[0]] + edge[2];
                }
            }
        }

        return Arrays.copyOf(dist, V);  // Retorna as distâncias excluindo o novo vértice de origem
    }

    // Função para implementar o Algoritmo de Johnson
    private static void johnsonAlgorithm(int[][] graph) {
        int V = graph.length;  // Número de vértices

        // Array para armazenar arestas
        // O número maximo de arestas
        // [3] origem, destino, aresta
        int[][] edges = new int[V * (V - 1) / 2][3];

        int index = 0;

        // Percorre a matriz de adjacencia do grafo para coletar todas as arestas do grafo
        // I tratado com origem
        // J tratado com destino
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                // Verifica se existe uma aresta saindo do vértice I e chegando em J
                if (graph[i][j] != 0) {
                    // Salva em edges I(origem), J(destino), (peso da aresta)
                    edges[index++] = new int[]{i, j, graph[i][j]};
                }
            }
        }

        // Obtem os pesos modificados para remover pesos negativos usando Bellman-Ford
        int[] alteredWeights = bellmanFordAlgorithm(edges, V);
        // Cria um grafoAlterado para percorrer atualizando os pesos
        int[][] alteredGraph = new int[V][V];

        // Percorre a matriz de adjacencia para
        // Modificar os pesos das arestas para garantir que todos os pesos sejam não-negativos
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (graph[i][j] != 0) {
                    alteredGraph[i][j] = graph[i][j] + alteredWeights[i] - alteredWeights[j];
                                      //Original   //Menor Caminho para I //Menor caminho para j
                }
            }
        }

        // Imprime o grafo modificado com arestas reponderadas
        System.out.println("Grafo Modificado:");
        for (int[] row : alteredGraph) {
            for (int weight : row) {
                System.out.print(weight + " ");
            }
            System.out.println();
        }

        // Executa o algoritmo de Dijkstra para cada vértice como origem
        for (int source = 0; source < V; source++) {
            System.out.println("\nMenor distância com o vértice " + source + " como origem:");
            dijkstraAlgorithm(graph, alteredGraph, source);
        }
    }
}
