#include "./OrganizedToCanon.h"
#include "DualTask.h"

void testCanon() {
    OrganizedToCanon canon(true);

    vector<double> goal = { -2, -3, 1 };
    canon.setGoal(goal);

    canon.addRestriction(new LowerStrongRestrict({ 1, 2, -1 }, 4));
    canon.addRestriction(new BiggerStrongRestrict({ -4, 1, 1 }, 1));
    canon.addRestriction(new EqwRestrict({ 3, -2, 0 }, 3));

    canon.addXCondition(new BiggerStrongRestrict({ 1, 0, 0 }, 0)); // x1 ≥ 0
    canon.addXCondition(new BiggerStrongRestrict({ 0, 1, 0 }, 0)); // x2 ≥ 0

    canon.print();
    canon.createCanonForm();
    canon.print();
}

void testRighnt() {
    vector<RestrictBase*> restricts;
    restricts.push_back(new LowerStrongRestrict({ 1, 2, 1 }, 6));
    restricts.push_back(new LowerStrongRestrict({ 2, 1, 3 }, 4));
    restricts.push_back(new EqwRestrict({ 1, 1, 1 }, 5));

    vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0 }, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0 }, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1 }, 0));
    vector<double> goal = { 3, 5, 2 };
    DirectTask* canon = new DirectTask(restricts, x_cond, goal, false);

    cout << endl << "original" << endl;
    canon->print();
    /*cout << endl << "canonical" << endl;
    canon->createCanonForm();
    canon->print();*/
    cout << endl << "against" << endl;
    DualTask* against = canon->goToAgainst();
    against->print();
}

int main() {
    testRighnt();


	return 0;
}