#pragma once
#include "./Utils.h"
#include "./Task.h"

class DualTask: public OrganizedToCanon {
public:
	DualTask(vector<RestrictBase*>& s_conditions, vector<RestrictBase*>& x_conditions, vector<double>& goal, bool isMin)
		: OrganizedToCanon(isMin){
		this->s_conditions = s_conditions;
		this->x_conditions = x_conditions;
		this->goal = goal;
		this->makeUnknownNatural();
	}

	virtual int getIndexMainColumn() = 0;

	virtual DualTask* goToAgainst() = 0;
};

class ReverseTask;

class DirectTask : public DualTask {
public:
	DirectTask(vector<RestrictBase*>& s_conditions, vector<RestrictBase*>& x_conditions, vector<double>& goal, bool isMin) : DualTask(s_conditions, x_conditions, goal, isMin) { // на максимум
		if (isMin) {
			Utils::invertGoal(goal);
			isMinTask = false;
		}
		for (int i = 0; i < this->s_conditions.size(); i++) {
			RestrictBase* lower = this->s_conditions[i]->toLower();
			if (lower != this->s_conditions[i])
				delete this->s_conditions[i];
			this->s_conditions[i] = lower;
		}
	}

	int getIndexMainColumn() override { // < 0
		int minElement = goal[0]; // Предположим, что первый элемент min

		for (int i = 1; i < goal.size(); ++i) {
			if (goal[i] < minElement) {
				minElement = goal[i];
			}
		}

		return minElement < 0 ? minElement : 0;
	}

	DualTask* goToAgainst() override;
};

class ReverseTask : public DualTask {
public:
	ReverseTask(vector<RestrictBase*>& s_conditions, vector<RestrictBase*>& x_conditions, vector<double>& goal, bool isMin) : DualTask(s_conditions, x_conditions, goal, isMin) {
		if (!isMin) {
			Utils::invertGoal(goal);
			isMinTask = true;
		}
		for (int i = 0; i < this->s_conditions.size(); i++) {
			RestrictBase* lower = this->s_conditions[i]->toBigger();
			if (lower != this->s_conditions[i])
				delete this->s_conditions[i];
			this->s_conditions[i] = lower;
		}
	}

	DualTask* goToAgainst() override {
		vector<vector<double>> transp_matr = Utils::transposeMatrix(Utils::getMatrFromConditions(s_conditions));
		vector<RestrictBase*> s_dual_conditions;
		vector<RestrictBase*> x_dual_conditions;
		for (int i = 0; i < goal.size(); i++) {
			s_dual_conditions.push_back(new LowerStrongRestrict(move(transp_matr[i]), goal[i])); // целевая функция - новые bi
		}
		vector<double> new_goal;
		for (int i = 0; i < s_conditions.size(); i++) {
			new_goal.push_back(s_conditions[i]->param);
		}
		for (int i = 0; i < new_goal.size(); i++) {
			vector<double> x(new_goal.size(), 0);
			x[i] = 1;
			x_dual_conditions.push_back(new BiggerStrongRestrict(move(x), 0));
		}
		return new DirectTask(s_dual_conditions, x_dual_conditions, new_goal, !isMinTask);
	}

	int getIndexMainColumn() override {
		int maxElement = goal[0]; // Предположим, что первый элемент максимален

		for (int i = 1; i < goal.size(); ++i) {
			if (goal[i] > maxElement) {
				maxElement = goal[i];
			}
		}

		return maxElement > 0 ? maxElement : 0;
	}
};

DualTask* DirectTask::goToAgainst() {
	vector<vector<double>> transp_matr = Utils::transposeMatrix(Utils::getMatrFromConditions(s_conditions)); // не меняет this
	vector<RestrictBase*> s_dual_conditions;
	vector<RestrictBase*> x_dual_conditions;
	for (int i = 0; i < goal.size(); i++) {
		s_dual_conditions.push_back(new BiggerStrongRestrict(move(transp_matr[i]), goal[i])); // целевая функция - новые bi
	}
	vector<double> new_goal;
	for (int i = 0; i < s_conditions.size(); i++) {
		new_goal.push_back(s_conditions[i]->param);
	}
	for (int i = 0; i < new_goal.size(); i++) {
		vector<double> x(new_goal.size(), 0);
		x[i] = 1;
		x_dual_conditions.push_back(new BiggerStrongRestrict(move(x), 0));
	}
	return new ReverseTask(s_dual_conditions, x_dual_conditions, new_goal, !isMinTask);
}