#include "./Task.h"

int main() {
    Task task;

    vector<double> goal = { -2, -3, 1 };
    task.setGoal(goal);

    task.addRestriction(new LowerStrongRestrict({ 1, 2, -1 }, 4));
    task.addRestriction(new BiggerStrongRestrict({ -4, 1, 1 }, 1));
    task.addRestriction(new EqwRestrict({ 3, -2, 0 }, 3));

    task.addXCondition(new BiggerStrongRestrict({ 1, 0, 0 }, 0)); // x1 ≥ 0
    task.addXCondition(new BiggerStrongRestrict({ 0, 1, 0 }, 0)); // x2 ≥ 0

    task.print();
    task.transformToCanonicalForm();
    task.print();
	return 0;
}