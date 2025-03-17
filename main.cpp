#include "./OrganizedToCanon.h"
#include "Simplex.h"

void testSimplex2() {
    // Создание ограничений
    std::vector<RestrictBase*> restricts;

    restricts.push_back(new EqwRestrict({ 2, 1, 3, 1, 2 }, 5)); // (x1 + 2x2 + x3 + x4 + x5 = 20)
    restricts.push_back(new EqwRestrict({ 1, 4, 0, 0, 3 }, 11)); // (3x1 + x2 + 2x3 + 2x4 + 4x5 = 40)
    restricts.push_back(new EqwRestrict({ 3, 2, 1, 0, 4 }, 8)); // (x1 + x2 + 3x3 + x4 + x5 = 30)
    restricts.push_back(new LowerStrongRestrict({ 1, -1, 2, 0, -1 }, 3)); // (x1 + 3x2 + 2x3 ≤ 25)
    restricts.push_back(new BiggerStrongRestrict({ -1, 2, -3, 0, 1 }, -4)); // (2x1 + x4 + 3x5 ≥ 15)

    // Ограничения на знак переменных
    std::vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0, 0 }, 0)); // x1 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0, 0, 0 }, 0)); // x2 ≥ 0
    x_cond.push_back(new LowerStrongRestrict({ 0, 0, 0, 1, 0 }, 0)); // x3 ≥ 0

    // Целевая функция
    std::vector<double> goal = { 4, 3, 7, 5, 6 }; // z = 4x1 + 3x2 + 5x3 + 2x4 + x5

    // Переменная, указывающая, что задача на максимизацию
    bool isMin = true; // isMin = false указывает на задачу максимизации

    OrganizedToCanon* task = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    task->print();
    Simplex simplex(task);
    cout << simplex.solve() << endl;
}

void testSimplex3() { // 105
    // Создание ограничений
    std::vector<RestrictBase*> restricts;

    restricts.push_back(new LowerStrongRestrict({ 1, 3, 5, 3 }, 40)); // (x1 + 2x2 + x3 + x4 + x5 = 20)
    restricts.push_back(new LowerStrongRestrict({ 2, 6, 1, 0 }, 50)); // (3x1 + x2 + 2x3 + 2x4 + 4x5 = 40)
    restricts.push_back(new LowerStrongRestrict({ 2, 3, 2, 5 }, 30)); // (x1 + x2 + 3x3 + x4 + x5 = 30)

    // Ограничения на знак переменных
    std::vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0}, 0)); // x1 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0, 0}, 0)); // x2 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1, 0}, 0)); // x3 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 0, 1 }, 0)); // x4 ≥ 0

    // Целевая функция
    std::vector<double> goal = { 7,8, 6, 5 }; // z = 4x1 + 3x2 + 5x3 + 2x4 + x5

    // Переменная, указывающая, что задача на максимизацию
    bool isMin = false; // isMin = false указывает на задачу максимизации

    OrganizedToCanon* task = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    Simplex simplex(task);
    cout << simplex.solve() << endl;
}

void testUniq() {
    std::vector<RestrictBase*> restricts;

    restricts.push_back(new LowerStrongRestrict({ 1, 3, 5, 3 }, 40)); // (x1 + 2x2 + x3 + x4 + x5 = 20)
    restricts.push_back(new LowerStrongRestrict({ 2, 6, 1, 0 }, 50)); // (3x1 + x2 + 2x3 + 2x4 + 4x5 = 40)
    restricts.push_back(new LowerStrongRestrict({ 2, 3, 2, 5 }, 30)); // (x1 + x2 + 3x3 + x4 + x5 = 30)

    // Ограничения на знак переменных
    std::vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0 }, 0)); // x1 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0, 0 }, 0)); // x2 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1, 0 }, 0)); // x3 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 0, 1 }, 0)); // x4 ≥ 0

    // Целевая функция
    std::vector<double> goal = { 7,8, 6, 5 }; // z = 4x1 + 3x2 + 5x3 + 2x4 + x5
    bool isMin = false; // isMin = false указывает на задачу максимизации

    OrganizedToCanon* task = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    OrganizedToCanon* task2 = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    task->createCanonForm();
    task->print();
    task2->print();
}

void testDual() {
    // Создание ограничений
    std::vector<RestrictBase*> restricts;

    restricts.push_back(new EqwRestrict({ 1, 3, 5, 3 }, 40)); // (x1 + 2x2 + x3 + x4 + x5 = 20)
    restricts.push_back(new LowerStrongRestrict({ 2, 6, 1, 0 }, 50)); // (3x1 + x2 + 2x3 + 2x4 + 4x5 = 40)
    restricts.push_back(new BiggerStrongRestrict({ 2, 3, 2, 5 }, 30)); // (x1 + x2 + 3x3 + x4 + x5 = 30)

    // Ограничения на знак переменных
    std::vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0 }, 0)); // x1 ≥ 0
    x_cond.push_back(new LowerStrongRestrict({ 0, 1, 0, 0 }, 0)); // x2 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1, 0 }, 0)); // x3 ≥ 0
    //x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 0, 1 }, 0)); // x4 ≥ 0

    // Целевая функция
    std::vector<double> goal = { 7,8, 6, 5 }; // z = 4x1 + 3x2 + 5x3 + 2x4 + x5

    // Переменная, указывающая, что задача на максимизацию
    bool isMin = false; // isMin = false указывает на задачу максимизации

    OrganizedToCanon* task = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    OrganizedToCanon* dual = task->goToAgainst();
    task->print();
    dual->print();
    /*Simplex simplex(task);
    cout << simplex.solve() << endl;*/


}

void testSimplexDual() {
    // Создание ограничений
    std::vector<RestrictBase*> restricts;

    restricts.push_back(new LowerStrongRestrict({ 1, 3, 5, 3 }, 40)); // (x1 + 2x2 + x3 + x4 + x5 = 20)
    restricts.push_back(new LowerStrongRestrict({ 2, 6, 1, 0 }, 50)); // (3x1 + x2 + 2x3 + 2x4 + 4x5 = 40)
    restricts.push_back(new LowerStrongRestrict({ 2, 3, 2, 5 }, 30)); // (x1 + x2 + 3x3 + x4 + x5 = 30)

    // Ограничения на знак переменных
    std::vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0 }, 0)); // x1 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0, 0 }, 0)); // x2 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1, 0 }, 0)); // x3 ≥ 0
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 0, 1 }, 0)); // x4 ≥ 0

    // Целевая функция
    std::vector<double> goal = { 7,8, 6, 5 }; // z = 4x1 + 3x2 + 5x3 + 2x4 + x5

    // Переменная, указывающая, что задача на максимизацию
    bool isMin = false; // isMin = false указывает на задачу максимизации

    OrganizedToCanon* task = new OrganizedToCanon(restricts, x_cond, goal, isMin);
    OrganizedToCanon* dual = task->goToAgainst();
    task->print();
    dual->print();
    Simplex simplex(task);
    cout << simplex.solve() << endl;
    Simplex simplex2(dual);
    cout << endl << "dual" << endl << simplex2.solve() << endl;


}

int main() {
    try {
        testSimplex3();
    }
    catch (exception& err) {
        cout << err.what();
    }


	return 0;
}