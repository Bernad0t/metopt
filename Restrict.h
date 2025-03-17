#pragma once
#include <vector>
#include <iostream>
#include <utility> // Для std::move

using namespace std;

enum Type {
	eqw,
	low,
	big
};


class EqwRestrict;
class BiggerStrongRestrict;

//class ConditionBase {
//protected:
//	vector<RestrictBase*> x_condition;
//public:
//
//}; // тут бы по-хорошему реализовать логику подстановки x_cond чтобы не хардкодить

class RestrictBase {
public:
	double param; // b
	vector<double> row_matr;
	RestrictBase(vector<double>&& row_matr, double param) : row_matr(std::move(row_matr)), param(param) {}
	virtual bool checkCorrect(vector<double> x) = 0;

	virtual ~RestrictBase() = default;
	virtual RestrictBase* clone() const = 0;


	virtual RestrictBase* multiplyOnNumber(double number) {
		param *= number;
		for (int i = 0; i < row_matr.size(); i++) {
			row_matr[i] *= number;
		}
		return this;
	}

	virtual EqwRestrict* toEqw() = 0;
	virtual RestrictBase* toLower() = 0;
	virtual RestrictBase* toBigger() = 0;
	virtual Type getType() = 0;

	int getIndexNotZeroValue() {
		for (int i = 0; i < row_matr.size(); i++) {
			if (row_matr[i] != 0)
				return i;
		}
		return -1;
	}

	virtual void print() {
		for (int i = 0; i < row_matr.size(); i++) {
			cout << row_matr[i] << " ";
		}
	}
};

class EqwRestrict : public RestrictBase { // ==
public:
	EqwRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res != param)
			return false;
		return true;
	}

	Type getType() override {
		return Type::eqw;
	}

	RestrictBase* multiplyOnNumber(double number) override {
		RestrictBase* res = RestrictBase::multiplyOnNumber(number);
		return new EqwRestrict(move(res->row_matr), res->param);
	}

	EqwRestrict* clone() const override {
		return new EqwRestrict(*this);
	}

	EqwRestrict* toEqw() override {
		return this;
	}

	RestrictBase* toLower() {
		return toEqw();
	}

	RestrictBase* toBigger() {
		return toEqw();
	}

	void print() {
		RestrictBase::print();
		cout << "==" << param << endl;
	}
};

class LowerStrongRestrict : public RestrictBase { // <=
public:
	LowerStrongRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res > param)
			return false;
		return true;
	}

	Type getType() override {
		return Type::low;
	}

	RestrictBase* multiplyOnNumber(double number) override;

	EqwRestrict* toEqw() override {
		row_matr.push_back(1);
		return new EqwRestrict(move(row_matr), param);
	}
	RestrictBase* toBigger() {
		return multiplyOnNumber(-1);
	}

	LowerStrongRestrict* clone() const override {
		return new LowerStrongRestrict(*this);
	}

	RestrictBase* toLower() {
		return this;
	}

	void print() {
		RestrictBase::print();
		cout << "<=" << param << endl;
	}
};

class BiggerStrongRestrict : public RestrictBase { // >=
public:
	BiggerStrongRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res < param)
			return false;
		return true;
	}

	RestrictBase* multiplyOnNumber(double number) override {
		RestrictBase* res = RestrictBase::multiplyOnNumber(number);
		if (number > 0)
			return new BiggerStrongRestrict(move(res->row_matr), res->param);
		else
			return new LowerStrongRestrict(move(res->row_matr), res->param);
	}

	Type getType() override {
		return Type::big;
	}

	BiggerStrongRestrict* clone() const override {
		return new BiggerStrongRestrict(*this);
	}

	EqwRestrict* toEqw() override {
		row_matr.push_back(-1);
		return new EqwRestrict(move(row_matr), param);
	}

	RestrictBase* toLower() {
		return multiplyOnNumber(-1);
	}

	RestrictBase* toBigger() {
		return this;
	}

	void print() {
		RestrictBase::print();
		cout << ">=" << param << endl;
	}
};

RestrictBase* LowerStrongRestrict::multiplyOnNumber(double number) {
	RestrictBase* res = RestrictBase::multiplyOnNumber(number);
	if (number > 0)
		return new LowerStrongRestrict(move(res->row_matr), res->param);
	else
		return new BiggerStrongRestrict(move(res->row_matr), res->param);
}