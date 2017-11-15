#include <cmath>
#include <fftw3.h>
#include <openvdb/openvdb.h>

void project_fft(int res[3], float *vel_x, float *vel_y, float *vel_z, float dt, float visc)
{
	fftwf_complex *vel_x_c = new fftwf_complex[res[0] * res[1] * res[2]];
	fftwf_complex *vel_y_c = new fftwf_complex[res[0] * res[1] * res[2]];
	fftwf_complex *vel_z_c = new fftwf_complex[res[0] * res[1] * res[2]];

	/* forward fft */
	fftwf_plan fwd_plan_x = fftwf_plan_dft_r2c_3d(res[0], res[1], res[2], vel_x, vel_x_c, FFTW_ESTIMATE);
	fftwf_plan fwd_plan_y = fftwf_plan_dft_r2c_3d(res[0], res[1], res[2], vel_y, vel_y_c, FFTW_ESTIMATE);
	fftwf_plan fwd_plan_z = fftwf_plan_dft_r2c_3d(res[0], res[1], res[2], vel_z, vel_z_c, FFTW_ESTIMATE);

	fftwf_execute(fwd_plan_x);
	fftwf_execute(fwd_plan_y);
	fftwf_execute(fwd_plan_z);

	fftwf_destroy_plan(fwd_plan_x);
	fftwf_destroy_plan(fwd_plan_y);
	fftwf_destroy_plan(fwd_plan_z);

	/* project velocity */
	float U[2], V[2], W[2];

	int index = 0;
	for (int z = 0; z < res[2]; ++z) {
		for (int y = 0; y < res[2]; ++y) {
			for (int x = 0; x < res[2]; ++x, ++index) {
				const float kk = x * x + y * y + z * z;

				if (kk == 0.0f) continue;

				const float f = std::exp(-kk * dt * visc);
				U[0] = vel_x_c[index][0];
				U[1] = vel_x_c[index][1];
				V[0] = vel_y_c[index][0];
				V[1] = vel_y_c[index][1];
				W[0] = vel_z_c[index][0];
				W[1] = vel_z_c[index][1];

				const float xxx = x * x * x / kk;
				const float yyy = y * y * y / kk;
				const float zzz = z * z * z / kk;
				const float xyz = x * y * z / kk;

				vel_x_c[index][0] = f * (1.0f - xxx) * U[0] - (     - xyz) * V[0] - (     - xyz) * W[0];
				vel_x_c[index][1] = f * (1.0f - xxx) * U[1] - (     - xyz) * V[1] - (     - xyz) * W[1];
				vel_y_c[index][0] = f * (     - xyz) * U[0] - (1.0f - yyy) * V[0] - (     - xyz) * W[0];
				vel_y_c[index][1] = f * (     - xyz) * U[1] - (1.0f - yyy) * V[1] - (     - xyz) * W[1];
				vel_z_c[index][0] = f * (     - xyz) * U[0] - (     - xyz) * V[0] - (1.0f - zzz) * W[0];
				vel_z_c[index][1] = f * (     - xyz) * U[1] - (     - xyz) * V[1] - (1.0f - zzz) * W[1];
			}
		}
	}

	/* backward fft */
	fftwf_plan bwd_plan_x = fftwf_plan_dft_c2r_3d(res[0], res[1], res[2], vel_x_c, vel_x, FFTW_ESTIMATE);
	fftwf_plan bwd_plan_y = fftwf_plan_dft_c2r_3d(res[0], res[1], res[2], vel_y_c, vel_y, FFTW_ESTIMATE);
	fftwf_plan bwd_plan_z = fftwf_plan_dft_c2r_3d(res[0], res[1], res[2], vel_z_c, vel_z, FFTW_ESTIMATE);

	fftwf_execute(bwd_plan_x);
	fftwf_execute(bwd_plan_y);
	fftwf_execute(bwd_plan_z);

	fftwf_destroy_plan(bwd_plan_x);
	fftwf_destroy_plan(bwd_plan_y);
	fftwf_destroy_plan(bwd_plan_z);

	/* copy back and normalize velocity */
	const float f = 1.0f / (res[0] * res[1] * res[2]);
	index = 0;
	for (int z = 0; z < res[2]; ++z) {
		for (int y = 0; y < res[2]; ++y) {
			for (int x = 0; x < res[2]; ++x, ++index) {
				vel_x[index] *= f;
				vel_y[index] *= f;
				vel_z[index] *= f;
			}
		}
	}
}
#if 0

void project_vdb_fft()
{
	using namespace openvdb;

	FloatGrid::Ptr grid;
	FloatGrid::Accessor acc = grid->getAccessor();
	FloatTree::LeafIter leaves;

	for (leaves = grid->tree().beginLeaf(); leaves; ++leaves) {
		tree::LeafNode leaf = *leaves;

		leaf.buffer().data();
	}
}
#endif
