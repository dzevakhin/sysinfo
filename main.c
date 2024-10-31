#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <linux/if_link.h>
#include <math.h>

#define MAX_PATH 256
#define MAX_CPUS 256
#define BUFFER_SIZE 256
#define UPDATE_INTERVAL 1  // seconds
#define TEMP_SENSOR_PATH "/sys/class/thermal/thermal_zone"
#define MAX_THERMAL_ZONES 20
#define HISTORY_SIZE 60    // 60 seconds of history
#define MAX_INTERFACES 16
#define GRAPH_HEIGHT 10
#define GRAPH_WIDTH 60

volatile sig_atomic_t stop = 0;

// Network interface statistics
typedef struct {
    char name[IF_NAMESIZE];
    unsigned long long rx_bytes;
    unsigned long long tx_bytes;
    unsigned long long rx_packets;
    unsigned long long tx_packets;
    unsigned long long rx_errors;
    unsigned long long tx_errors;
    unsigned long long rx_dropped;
    unsigned long long tx_dropped;
    // Calculated rates
    double rx_rate;  // bytes per second
    double tx_rate;  // bytes per second
    double rx_packets_rate;  // packets per second
    double tx_packets_rate;  // packets per second
    // History for graphs
    double rx_history[GRAPH_WIDTH];
    double tx_history[GRAPH_WIDTH];
} Network_Stats;

typedef struct {
    int cpu_id;
    int physical_id;    // Socket ID
    int core_id;
    char *thread_siblings;
    double current_freq;  // MHz
    double max_freq;      // MHz
    double min_freq;      // MHz
    double usage;         // CPU usage percentage
    int cache_L1d;       // L1 Data cache size (KB)
    int cache_L1i;       // L1 Instruction cache size (KB)
    int cache_L2;        // L2 cache size (KB)
    int cache_L3;        // L3 cache size (KB)
    int temperature;     // Temperature in Celsius
    char power_state[20];// Current C-state
    unsigned long long interrupts; // Number of interrupts
    double power_draw;   // Power consumption in Watts (if available)
    // History for graphs
    double usage_history[GRAPH_WIDTH];
    double temp_history[GRAPH_WIDTH];
} CPU_Info;

// Previous CPU stats for usage calculation
typedef struct {
    unsigned long long user;
    unsigned long long nice;
    unsigned long long system;
    unsigned long long idle;
    unsigned long long iowait;
    unsigned long long irq;
    unsigned long long softirq;
    unsigned long long steal;
} CPU_Stats;

// Global variables
CPU_Stats prev_stats[MAX_CPUS];
Network_Stats net_stats[MAX_INTERFACES];
int net_count = 0;
struct winsize term_size;

// Function to format bytes to human readable format
void format_bytes(double bytes, char *buffer) {
    const char* units[] = {"B", "KB", "MB", "GB", "TB"};
    int unit = 0;
    while (bytes >= 1024 && unit < 4) {
        bytes /= 1024;
        unit++;
    }
    sprintf(buffer, "%.1f %s", bytes, units[unit]);
}

// Draw ASCII graph
void draw_graph(double *history, int width, int height, double max_value, char *buffer) {
    char graph[GRAPH_HEIGHT][GRAPH_WIDTH + 1] = {0};
    int i, j;

    // Initialize graph with spaces
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            graph[i][j] = ' ';
        }
        graph[i][width] = '\0';
    }

    // Draw graph lines
    for (j = 0; j < width; j++) {
        if (max_value > 0) {
            int val = (int)((history[j] / max_value) * (height - 1));
            for (i = height - 1; i >= height - 1 - val && i >= 0; i--) {
                graph[i][j] = '|';
            }
        }
    }

    // Combine into output buffer
    buffer[0] = '\0';
    for (i = 0; i < height; i++) {
        strcat(buffer, graph[i]);
        strcat(buffer, "\n");
    }
}

// Signal handler for graceful exit
void handle_signal(int signum) {
    stop = 1;
}

// Get terminal size
void update_terminal_size(void) {
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &term_size);
}

// Read CPU temperature
int read_cpu_temperature(int cpu_id) {
    char path[MAX_PATH];
    char buffer[BUFFER_SIZE];
    FILE *fp;
    int temp = -1;

    // Try coretemp
    snprintf(path, sizeof(path),
             "/sys/devices/platform/coretemp.%d/hwmon/hwmon*/temp%d_input",
             cpu_id/4, (cpu_id%4)+1);
    fp = fopen(path, "r");
    if (fp) {
        if (fscanf(fp, "%d", &temp) == 1) {
            fclose(fp);
            return temp / 1000;
        }
        fclose(fp);
    }

    // Try thermal_zone
    for (int i = 0; i < MAX_THERMAL_ZONES; i++) {
        snprintf(path, sizeof(path),
                 "/sys/class/thermal/thermal_zone%d/temp", i);
        fp = fopen(path, "r");
        if (fp) {
            if (fscanf(fp, "%d", &temp) == 1) {
                fclose(fp);
                return temp / 1000;
            }
            fclose(fp);
        }
    }

    return temp;
}

// Read network statistics
void update_network_stats(void) {
    struct ifaddrs *ifaddr, *ifa;
    static struct timespec last_update = {0, 0};
    struct timespec current_time;
    double time_diff;

    clock_gettime(CLOCK_MONOTONIC, &current_time);
    time_diff = (current_time.tv_sec - last_update.tv_sec) +
                (current_time.tv_nsec - last_update.tv_nsec) / 1e9;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return;
    }

    // Store previous values for rate calculation
    unsigned long long prev_rx_bytes[MAX_INTERFACES];
    unsigned long long prev_tx_bytes[MAX_INTERFACES];
    unsigned long long prev_rx_packets[MAX_INTERFACES];
    unsigned long long prev_tx_packets[MAX_INTERFACES];

    for (int i = 0; i < net_count; i++) {
        prev_rx_bytes[i] = net_stats[i].rx_bytes;
        prev_tx_bytes[i] = net_stats[i].tx_bytes;
        prev_rx_packets[i] = net_stats[i].rx_packets;
        prev_tx_packets[i] = net_stats[i].tx_packets;
    }

    // Reset counter for new scan
    net_count = 0;

    for (ifa = ifaddr; ifa != NULL && net_count < MAX_INTERFACES; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL || ifa->ifa_addr->sa_family != AF_PACKET)
            continue;

        struct rtnl_link_stats *stats = ifa->ifa_data;
        if (!stats)
            continue;

        // Skip loopback and inactive interfaces
        if ((ifa->ifa_flags & IFF_LOOPBACK) || !(ifa->ifa_flags & IFF_UP))
            continue;

        strncpy(net_stats[net_count].name, ifa->ifa_name, IF_NAMESIZE - 1);
        net_stats[net_count].rx_bytes = stats->rx_bytes;
        net_stats[net_count].tx_bytes = stats->tx_bytes;
        net_stats[net_count].rx_packets = stats->rx_packets;
        net_stats[net_count].tx_packets = stats->tx_packets;
        net_stats[net_count].rx_errors = stats->rx_errors;
        net_stats[net_count].tx_errors = stats->tx_errors;
        net_stats[net_count].rx_dropped = stats->rx_dropped;
        net_stats[net_count].tx_dropped = stats->tx_dropped;

        // Calculate rates if we have previous values
        if (time_diff > 0) {
            net_stats[net_count].rx_rate =
                (net_stats[net_count].rx_bytes - prev_rx_bytes[net_count]) / time_diff;
            net_stats[net_count].tx_rate =
                (net_stats[net_count].tx_bytes - prev_tx_bytes[net_count]) / time_diff;
            net_stats[net_count].rx_packets_rate =
                (net_stats[net_count].rx_packets - prev_rx_packets[net_count]) / time_diff;
            net_stats[net_count].tx_packets_rate =
                (net_stats[net_count].tx_packets - prev_tx_packets[net_count]) / time_diff;
        }

        // Shift history and add new values
        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            net_stats[net_count].rx_history[i] = net_stats[net_count].rx_history[i + 1];
            net_stats[net_count].tx_history[i] = net_stats[net_count].tx_history[i + 1];
        }
        net_stats[net_count].rx_history[GRAPH_WIDTH - 1] = net_stats[net_count].rx_rate;
        net_stats[net_count].tx_history[GRAPH_WIDTH - 1] = net_stats[net_count].tx_rate;

        net_count++;
    }

    freeifaddrs(ifaddr);
    last_update = current_time;
}

// Previous CPU-related functions remain the same...
void read_cpu_power_state(int cpu_id, char *state) {
    char path[MAX_PATH];
    FILE *fp;

    snprintf(path, sizeof(path),
             "/sys/devices/system/cpu/cpu%d/cpuidle/current_driver",
             cpu_id);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(state, 20, fp) != NULL) {
            state[strcspn(state, "\n")] = 0;
        } else {
            strcpy(state, "unknown");
        }
        fclose(fp);
    } else {
        strcpy(state, "unknown");
    }
}

double read_cpu_power(int cpu_id) {
    char path[MAX_PATH];
    char buffer[BUFFER_SIZE];
    FILE *fp;
    double power = -1;

    snprintf(path, sizeof(path),
             "/sys/class/powercap/intel-rapl/intel-rapl:%d/energy_uj",
             cpu_id/4);
    fp = fopen(path, "r");
    if (fp) {
        if (fscanf(fp, "%lf", &power) == 1) {
            fclose(fp);
            return power / 1000000.0;
        }
        fclose(fp);
    }

    return power;
}

unsigned long long read_cpu_interrupts(int cpu_id) {
    FILE *fp;
    char *line = NULL;
    size_t len = 0;
    unsigned long long total_interrupts = 0;

    fp = fopen("/proc/interrupts", "r");
    if (!fp) return 0;

    // Skip header line
    getline(&line, &len, fp);

    while (getline(&line, &len, fp) != -1) {
        char *token;
        int cpu_count = 0;
        token = strtok(line, " ");

        while (token != NULL) {
            if (cpu_count == cpu_id + 1) {
                unsigned long long irq_count = strtoull(token, NULL, 10);
                total_interrupts += irq_count;
                break;
            }
            token = strtok(NULL, " ");
            cpu_count++;
        }
    }

    free(line);
    fclose(fp);
    return total_interrupts;
}

void read_cpu_file(int cpu_id, const char *base_path, const char *file_name, char *result) {
    char path[MAX_PATH];
    FILE *fp;

    snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/%s/%s",
             cpu_id, base_path, file_name);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(result, BUFFER_SIZE, fp) != NULL) {
            result[strcspn(result, "\n")] = 0;
        }
        fclose(fp);
    } else {
        strcpy(result, "N/A");
    }
}

void read_topology_file(int cpu_id, const char *file_name, char *result) {
    read_cpu_file(cpu_id, "topology", file_name, result);
}

int read_cache_size(int cpu_id, int level, char type) {
    char path[MAX_PATH];
    char buffer[BUFFER_SIZE];
    FILE *fp;

    snprintf(path, sizeof(path),
             "/sys/devices/system/cpu/cpu%d/cache/index%d/size",
             cpu_id, level);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp) != NULL) {
            int size;
            char unit[3];
            sscanf(buffer, "%d%2s", &size, unit);
            if (strcmp(unit, "M\n") == 0 || strcmp(unit, "MB") == 0) {
                size *= 1024;
            }
            fclose(fp);
            return size;
        }
        fclose(fp);
    }
    return -1;
}

double read_cpu_freq(int cpu_id, const char *freq_type) {
    char buffer[BUFFER_SIZE];
    char path[MAX_PATH];
    FILE *fp;

    snprintf(path, sizeof(path),
             "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_%s_freq",
             cpu_id, freq_type);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp) != NULL) {
            double freq = atol(buffer) / 1000.0;
            fclose(fp);
            return freq;
        }
        fclose(fp);
    }
    return -1;
}

double get_cpu_usage(int cpu_id) {
    static unsigned long long prev_total[MAX_CPUS] = {0};
    static unsigned long long prev_idle[MAX_CPUS] = {0};

    char buffer[BUFFER_SIZE];
    char cpu_label[10];
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    unsigned long long total, idle_time, used;
    double cpu_use;
    FILE *fp;

    fp = fopen("/proc/stat", "r");
    if (!fp) return -1;

    fgets(buffer, sizeof(buffer), fp);

    snprintf(cpu_label, sizeof(cpu_label), "cpu%d", cpu_id);
    while (fgets(buffer, sizeof(buffer), fp)) {
        if (strncmp(buffer, cpu_label, strlen(cpu_label)) == 0) {
            sscanf(buffer, "cpu%*d %llu %llu %llu %llu %llu %llu %llu %llu",
                   &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);

            idle_time = idle + iowait;
            total = user + nice + system + idle_time + irq + softirq + steal;

            if (prev_total[cpu_id] != 0) {
                used = (total - prev_total[cpu_id]) - (idle_time - prev_idle[cpu_id]);
                cpu_use = (double)used / (total - prev_total[cpu_id]) * 100.0;
            } else {
                cpu_use = 0.0;
            }

            prev_total[cpu_id] = total;
            prev_idle[cpu_id] = idle_time;

            fclose(fp);
            return cpu_use;
        }
    }

    fclose(fp);
    return -1;
}

int compare_cpus(const void *a, const void *b) {
    const CPU_Info *cpu_a = (const CPU_Info *)a;
    const CPU_Info *cpu_b = (const CPU_Info *)b;

    if (cpu_a->physical_id != cpu_b->physical_id)
        return cpu_a->physical_id - cpu_b->physical_id;
    if (cpu_a->core_id != cpu_b->core_id)
        return cpu_a->core_id - cpu_b->core_id;
    return cpu_a->cpu_id - cpu_b->cpu_id;
}

int starts_with(const char *str, const char *prefix) {
    if (str == NULL || prefix == NULL) {
        return 0;
    }
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

void print_network_stats() {
    printf("\n\033[1;36m=== Network Statistics ===\033[0m\n");

    for (int i = 0; i < net_count; i++) {
        if (starts_with(net_stats[i].name, "br") || starts_with(net_stats[i].name, "veth")) {
            continue;
        }

        char rx_str[32], tx_str[32];
        format_bytes(net_stats[i].rx_rate, rx_str);
        format_bytes(net_stats[i].tx_rate, tx_str);

        printf("\n\033[1;33mInterface: %s\033[0m\n", net_stats[i].name);
        printf("RX: %s/s (%0.1f pkts/s) [Errors: %llu, Dropped: %llu]\n",
               rx_str, net_stats[i].rx_packets_rate,
               net_stats[i].rx_errors, net_stats[i].rx_dropped);
        printf("TX: %s/s (%0.1f pkts/s) [Errors: %llu, Dropped: %llu]\n",
               tx_str, net_stats[i].tx_packets_rate,
               net_stats[i].tx_errors, net_stats[i].tx_dropped);

        /*
        // Draw RX/TX graphs
        char graph_buffer[GRAPH_HEIGHT * (GRAPH_WIDTH + 1) + 1];
        double max_rate = 0;

        // Find maximum rate for scaling
        for (int j = 0; j < GRAPH_WIDTH; j++) {
            if (net_stats[i].rx_history[j] > max_rate)
                max_rate = net_stats[i].rx_history[j];
            if (net_stats[i].tx_history[j] > max_rate)
                max_rate = net_stats[i].tx_history[j];
        }

        printf("\nRX Graph (max: ");
        format_bytes(max_rate, rx_str);
        printf("%s/s):\n", rx_str);
        draw_graph(net_stats[i].rx_history, GRAPH_WIDTH, GRAPH_HEIGHT, max_rate, graph_buffer);
        printf("%s", graph_buffer);

        printf("\nTX Graph (max: ");
        format_bytes(max_rate, tx_str);
        printf("%s/s):\n", tx_str);
        draw_graph(net_stats[i].tx_history, GRAPH_WIDTH, GRAPH_HEIGHT, max_rate, graph_buffer);
        printf("%s", graph_buffer);
    */
    }
}

void print_cpu_info(CPU_Info *cpus, int cpu_count) {
    static int iteration = 0;
    iteration++;

    // Clear screen and move cursor to top
    printf("\033[2J\033[H");

    // Print header with timestamp
    time_t now = time(NULL);
    char timestamp[26];
    ctime_r(&now, timestamp);
    timestamp[24] = '\0';

    printf("\033[1;36m=== CPU Monitoring System === %s (Update: %d) ===\033[0m\n\n",
           timestamp, iteration);

    // Print CPU information by socket
    int current_socket = -1;
    for (int i = 0; i < cpu_count; i++) {
        if (current_socket != cpus[i].physical_id) {
            current_socket = cpus[i].physical_id;
            printf("\n\033[1;33m=== Socket %d ===\033[0m\n", current_socket);

            // Print usage graph for this socket
            char graph_buffer[GRAPH_HEIGHT * (GRAPH_WIDTH + 1) + 1];
            double socket_usage[GRAPH_WIDTH] = {0};
            int core_count = 0;

            // Calculate average usage for all cores in this socket
            for (int j = i; j < cpu_count && cpus[j].physical_id == current_socket; j++) {
                for (int k = 0; k < GRAPH_WIDTH; k++) {
                    socket_usage[k] += cpus[j].usage_history[k];
                }
                core_count++;
            }

            // Average the values
            for (int k = 0; k < GRAPH_WIDTH; k++) {
                socket_usage[k] /= core_count;
            }

            printf("\nSocket Usage Graph (%%): \n");

            // Header for CPU info
            printf("\n%-4s %-6s %-6s %-8s %-10s %-8s %-8s %-6s %-6s %-10s %-8s\n",
                   "CPU", "Core", "Usage", "Temp", "Freq", "Power", "IRQs", "C-state",
                   "L1d/i", "L2", "L3");
            printf("%s\n", "----------------------------------------"
                   "----------------------------------------");
        }

        // Determine if this is a HyperThread
        int is_ht = 0;
        for (int j = 0; j < i; j++) {
            if (cpus[j].physical_id == cpus[i].physical_id &&
                cpus[j].core_id == cpus[i].core_id) {
                is_ht = 1;
                break;
            }
        }

        // Color coding based on temperature and usage
        const char *temp_color = "\033[0m";
        if (cpus[i].temperature >= 80) temp_color = "\033[1;31m";
        else if (cpus[i].temperature >= 70) temp_color = "\033[1;33m";

        const char *usage_color = "\033[0m";
        if (cpus[i].usage >= 90) usage_color = "\033[1;31m";
        else if (cpus[i].usage >= 70) usage_color = "\033[1;33m";

        // Print CPU info
        const char *core_color = is_ht ? "\033[1;35m" : "\033[1;32m";
        printf("%s%-4d %-6d%s %-6.1f%%%s %-3d°C%s %7.0fMHz %6.1fW %8llu %-8s %4d/%d %8d %8d\033[0m\n",
               core_color, cpus[i].cpu_id, cpus[i].core_id,
               usage_color, cpus[i].usage,
               temp_color, cpus[i].temperature,
               "\033[0m",
               cpus[i].current_freq,
               cpus[i].power_draw,
               cpus[i].interrupts,
               cpus[i].power_state,
               cpus[i].cache_L1d, cpus[i].cache_L1i,
               cpus[i].cache_L2,
               cpus[i].cache_L3);

        // Print individual core usage graph if terminal is wide enough
        /*
        if (term_size.ws_col >= 120) {
            char graph_buffer[GRAPH_HEIGHT * (GRAPH_WIDTH + 1) + 1];
            draw_graph(cpus[i].usage_history, GRAPH_WIDTH, 5, 100.0, graph_buffer);
            printf("%s", graph_buffer);
        }
    */
    }

    // Print network statistics
    print_network_stats();

    // Print legend
    printf("\n\033[1;36mLegend:\033[0m\n");
    printf("\033[1;32m█\033[0m Physical Core  ");
    printf("\033[1;35m█\033[0m HyperThread  ");
    printf("\033[1;31m█\033[0m High Load/Temp  ");
    printf("\033[1;33m█\033[0m Warning Load/Temp\n");

    // Print hotkeys
 printf("\nHotkeys: 'q' to quit, 's' to sort by usage, 't' to sort by temperature\n");
}

void update_cpu_info(CPU_Info *cpus, int cpu_count) {
    for (int i = 0; i < cpu_count; i++) {
        // Get current values
        cpus[i].current_freq = read_cpu_freq(cpus[i].cpu_id, "cur");
        cpus[i].usage = get_cpu_usage(cpus[i].cpu_id);
        cpus[i].temperature = read_cpu_temperature(cpus[i].cpu_id);
        cpus[i].interrupts = read_cpu_interrupts(cpus[i].cpu_id);
        cpus[i].power_draw = read_cpu_power(cpus[i].cpu_id);
        read_cpu_power_state(cpus[i].cpu_id, cpus[i].power_state);

        // Update history arrays
        for (int j = 0; j < GRAPH_WIDTH - 1; j++) {
            cpus[i].usage_history[j] = cpus[i].usage_history[j + 1];
            cpus[i].temp_history[j] = cpus[i].temp_history[j + 1];
        }
        cpus[i].usage_history[GRAPH_WIDTH - 1] = cpus[i].usage;
        cpus[i].temp_history[GRAPH_WIDTH - 1] = cpus[i].temperature;
    }

    // Update network statistics
    update_network_stats();
}

int init_cpu_info(CPU_Info *cpus) {
    DIR *dir;
    struct dirent *entry;
    int cpu_count = 0;
    char buffer[BUFFER_SIZE];

    // Open the CPU directory
    dir = opendir("/sys/devices/system/cpu");
    if (!dir) {
        perror("Cannot open CPU directory");
        return -1;
    }

    // Read CPU information
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "cpu", 3) == 0) {
            int cpu_id;
            if (sscanf(entry->d_name + 3, "%d", &cpu_id) == 1) {
                cpus[cpu_count].cpu_id = cpu_id;

                // Basic topology
                read_topology_file(cpu_id, "physical_package_id", buffer);
                cpus[cpu_count].physical_id = atoi(buffer);

                read_topology_file(cpu_id, "core_id", buffer);
                cpus[cpu_count].core_id = atoi(buffer);

                read_topology_file(cpu_id, "thread_siblings_list", buffer);
                cpus[cpu_count].thread_siblings = strdup(buffer);

                // Frequencies
                cpus[cpu_count].max_freq = read_cpu_freq(cpu_id, "max");
                cpus[cpu_count].min_freq = read_cpu_freq(cpu_id, "min");

                // Cache information
                cpus[cpu_count].cache_L1d = read_cache_size(cpu_id, 0, 'd');
                cpus[cpu_count].cache_L1i = read_cache_size(cpu_id, 1, 'i');
                cpus[cpu_count].cache_L2 = read_cache_size(cpu_id, 2, 'u');
                cpus[cpu_count].cache_L3 = read_cache_size(cpu_id, 3, 'u');

                // Initialize history arrays
                memset(cpus[cpu_count].usage_history, 0, sizeof(cpus[cpu_count].usage_history));
                memset(cpus[cpu_count].temp_history, 0, sizeof(cpus[cpu_count].temp_history));

                cpu_count++;
                if (cpu_count >= MAX_CPUS) break;
            }
        }
    }
    closedir(dir);

    // Sort CPUs
    qsort(cpus, cpu_count, sizeof(CPU_Info), compare_cpus);

    return cpu_count;
}

// Функция сравнения по загрузке
int compare_cpu_by_usage(const void *a, const void *b) {
    const CPU_Info *cpu_a = (const CPU_Info *)a;
    const CPU_Info *cpu_b = (const CPU_Info *)b;
    if (cpu_b->usage > cpu_a->usage) return 1;
    if (cpu_b->usage < cpu_a->usage) return -1;
    return 0;
}

// Функция сравнения по температуре
int compare_cpu_by_temperature(const void *a, const void *b) {
    const CPU_Info *cpu_a = (const CPU_Info *)a;
    const CPU_Info *cpu_b = (const CPU_Info *)b;
    return cpu_b->temperature - cpu_a->temperature;
}

int main() {
    // Set up signal handler
    signal(SIGINT, handle_signal);

    // Initialize CPU info array
    CPU_Info cpus[MAX_CPUS] = {0};
    int cpu_count = init_cpu_info(cpus);
    if (cpu_count < 0) {
        fprintf(stderr, "Failed to initialize CPU info\n");
        return 1;
    }

    // Set terminal to raw mode
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // Make stdin non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // Initialize network statistics
    update_network_stats();

    // Main monitoring loop
    while (!stop) {
        // Update terminal size
        update_terminal_size();

        // Update all information
        update_cpu_info(cpus, cpu_count);

        // Print information
        print_cpu_info(cpus, cpu_count);

        // Check for user input
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch (c) {
                case 'q':
                    stop = 1;
                break;
                case 's':
                    // Sort by usage
                        qsort(cpus, cpu_count, sizeof(CPU_Info), compare_cpu_by_usage);
                break;
                case 't':
                    // Sort by temperature
                        qsort(cpus, cpu_count, sizeof(CPU_Info), compare_cpu_by_temperature);
                break;
            }
        }

        sleep(UPDATE_INTERVAL);
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

    // Cleanup
    for (int i = 0; i < cpu_count; i++) {
        free(cpus[i].thread_siblings);
    }

    printf("\n\033[1;36mMonitoring stopped.\033[0m\n");
    return 0;
}