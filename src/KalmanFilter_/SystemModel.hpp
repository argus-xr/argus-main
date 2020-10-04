#ifndef KALMANSYSTEMMODEL_H
#define KALMANSYSTEMMODEL_H

#include <kalman/LinearizedSystemModel.hpp>

namespace ArgusKalman {

	class State : public Kalman::Vector<float, 13>
	{
	public:
		KALMAN_VECTOR(State, float, 13)

		// position
		static constexpr size_t pX = 0;
		static constexpr size_t pY = 1;
		static constexpr size_t pZ = 2;

		// velocity
		static constexpr size_t vX = 3;
		static constexpr size_t vY = 4;
		static constexpr size_t vZ = 5;

		// linear acceleration
		static constexpr size_t aX = 6;
		static constexpr size_t aY = 7;
		static constexpr size_t aZ = 8;

		// orientation quaternion
		static constexpr size_t oX = 9;
		static constexpr size_t oY = 10;
		static constexpr size_t oZ = 11;
		static constexpr size_t oW = 12;

		float px()       const { return (*this)[pX]; }
		float py()       const { return (*this)[pY]; }
		float pz()       const { return (*this)[pZ]; }

		float vx()       const { return (*this)[vX]; }
		float vy()       const { return (*this)[vY]; }
		float vz()       const { return (*this)[vZ]; }

		float ax()       const { return (*this)[aX]; }
		float ay()       const { return (*this)[aY]; }
		float az()       const { return (*this)[aZ]; }

		float ox()       const { return (*this)[oX]; }
		float oy()       const { return (*this)[oY]; }
		float oz()       const { return (*this)[oZ]; }
		float ow()       const { return (*this)[oW]; }

		float& px() { return (*this)[pX]; }
		float& py() { return (*this)[pY]; }
		float& pz() { return (*this)[pZ]; }

		float& vx() { return (*this)[vX]; }
		float& vy() { return (*this)[vY]; }
		float& vz() { return (*this)[vZ]; }

		float& ax() { return (*this)[aX]; }
		float& ay() { return (*this)[aY]; }
		float& az() { return (*this)[aZ]; }

		float& ox() { return (*this)[oX]; }
		float& oy() { return (*this)[oY]; }
		float& oz() { return (*this)[oZ]; }
		float& ow() { return (*this)[oW]; }
	};

	class Control : public Kalman::Vector<float, 0>
	{
	public:
		KALMAN_VECTOR(Control, float, 0)
	};

	class SystemModel : public Kalman::SystemModel<State, Control, Kalman::StandardBase>
	{
	public:

		State f(const State& x, const Control& c) const {
			State x_; // new state after this step

			x_.px() = x.px() + x.vx() + x.ax() * 0.5f;
			x_.py() = x.py() + x.vy() + x.ay() * 0.5f;
			x_.pz() = x.pz() + x.vz() + x.az() * 0.5f;

			x_.vx() = x.vx() + x.ax();
			x_.vy() = x.vy() + x.ay();
			x_.vz() = x.vz() + x.az();

			return x_;
		}
	};

}

#endif // KALMANSYSTEMMODEL_H