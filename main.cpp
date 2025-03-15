#include "./OrganizedToCanon.h"
#include "DualTask.h"
#include "Simplex.h"

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

void testSimplex() {
    vector<RestrictBase*> restricts;
    restricts.push_back(new LowerStrongRestrict({ 4, 1 }, 8));
    restricts.push_back(new BiggerStrongRestrict({ 1, -1 }, -3));
    vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0}, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1}, 0));

    vector<double> goal = { 3, 4 };
    DualTask* canon = new DirectTask(restricts, x_cond, goal, false);
    canon->print();

    /*Simplex simplex(canon);
    cout << simplex.solve() << endl;*/

    DualTask* inv = canon->goToAgainst();
    inv->print();

    Simplex simplex2(inv);
    cout << simplex2.solve() << endl;
}

void testSimplex2() {
    vector<RestrictBase*> restricts;
    restricts.push_back(new LowerStrongRestrict({ 2, 1, -3, 0 }, 0));
    restricts.push_back(new LowerStrongRestrict({ 0, 1, 2, -1 }, 7));
    restricts.push_back(new LowerStrongRestrict({ 3, 0, -2, -2 }, 4));
    restricts.push_back(new LowerStrongRestrict({ 1, 1, 1, 1 }, 17));
    vector<RestrictBase*> x_cond;
    x_cond.push_back(new BiggerStrongRestrict({ 1, 0, 0, 0 }, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 1, 0, 0 }, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 1, 0 }, 0));
    x_cond.push_back(new BiggerStrongRestrict({ 0, 0, 0, 1 }, 0));

    vector<double> goal = { 1, -2, 1, 2 };
    OrganizedToCanon* canon = new OrganizedToCanon(restricts, x_cond, goal, true);
    canon->print();

    Simplex simplex(canon);
    try {
        cout << simplex.solve() << endl;
    }
    catch (exception& err){
        cout << err.what();
    }
}

int main() {
    try {
        testSimplex();
    }
    catch (exception& err) {
        cout << err.what();
    }


	return 0;
}