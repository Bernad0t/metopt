#include "./LOF.h"
#include "./Forest.h"

void testLOF() {
    // Пример данных
    std::vector<std::vector<double>> inputData = {
        {1.0, 1.0},
        {1.1, 1.0},
        {1.0, 1.1},
        {2.0, 2.0},
        {10.0, 10.0}  // Аномалия
    };

    size_t k = 3;
    LOF lof(inputData, k);
    auto scores = lof.computeAllLOFs();

    std::cout << "LOF scores:\n";
    for (size_t i = 0; i < scores.size(); ++i) {
        std::cout << "Point " << i << ": " << scores[i] << "\n";
    }
}

void testFastLOF() {
    // Пример данных
    std::vector<std::vector<double>> inputData = {
        {1.0, 1.0},
        {1.1, 1.0},
        {1.0, 1.1},
        {2.0, 2.0},
        {10.0, 10.0},  // Аномалия
        {2.1, 2.0},
        {2.0, 2.1},
        {1.2, 1.0}
    };

    size_t k = 5;
    FastLOF fastlof(inputData, k);
    auto scores = fastlof.computeAllLOFs();

    std::cout << "FastLOF scores (with custom KD-tree):\n";
    for (size_t i = 0; i < scores.size(); ++i) {
        std::cout << "Point " << i << ": " << scores[i] << "\n";
    }
}

void testForest() {
    // Тестовые данные
    std::vector<std::vector<double>> data = {
        {1.0, 1.0}, {1.1, 1.0}, {1.0, 1.1},
        {2.0, 2.0}, {2.1, 2.0}, {2.0, 2.1},
        {3.0, 3.0}, {3.1, 3.0}, {3.0, 3.1},
        {10.0, 10.0}  // Явная аномалия
    };

    try {
        // Параметры для маленького датасета
        IsolationForest forest(50, 4, 5);  // 50 деревьев, подвыборка 4, глубина 5
        forest.fit(data);

        auto scores = forest.anomalyScores(data);

        // Вычисляем порог через IQR
        std::vector<double> sorted_scores = scores;
        std::sort(sorted_scores.begin(), sorted_scores.end());

        double q1 = sorted_scores[sorted_scores.size() / 4];
        double q3 = sorted_scores[sorted_scores.size() * 3 / 4];
        double iqr = q3 - q1;
        double threshold = q3 + 1.5 * iqr;

        std::cout << "Anomaly detection results:\n";
        std::cout << "Threshold: " << threshold << "\n\n";

        for (size_t i = 0; i < data.size(); ++i) {
            std::cout << "Point (" << data[i][0] << ", " << data[i][1] << "): "
                << scores[i];

            if (scores[i] > threshold) {
                std::cout << " <-- ANOMALY DETECTED";
            }
            std::cout << "\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return;
    }
}

int main() {
    testLOF();
    testFastLOF();
    testForest();
    return 0;
}