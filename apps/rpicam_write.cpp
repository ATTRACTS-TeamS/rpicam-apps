#include <chrono>

#include "core/rpicam_app.hpp"
#include "core/still_options.hpp"
#include "post_processing_stages/object_detect.hpp"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>

using namespace std::placeholders;
using libcamera::Stream;

sem_t *sem_read_;
sem_t *sem_write_;
bool init_shared_memory_ = false;
size_t memory_size_ = 0;
int shm_fd_ = -1;
const char *shm_name_ = "/rpicam_apps_shm";
const char *sem_read_name_ = "/rpicam_apps_sem_read";
const char *sem_write_name_ = "/rpicam_apps_sem_write";
unsigned char *shm_ptr_ = nullptr;

class RPiCamWriteApp : public RPiCamApp
{
public:
	RPiCamWriteApp() : RPiCamApp(std::make_unique<StillOptions>()) {}
	StillOptions *GetOptions() const { return static_cast<StillOptions *>(options_.get()); }
};

static bool init_shared_memory(size_t size)
{
	if (init_shared_memory_)
	{
		return true;
	}
	else
	{
		shm_fd_ = shm_open(shm_name_, O_CREAT | O_RDWR, 0666);
		if (shm_fd_ == -1)
		{
			LOG_ERROR("failed to create shared memory object");
			return false;
		}

		if (ftruncate(shm_fd_, size) == -1)
		{
			close(shm_fd_);
			shm_unlink(shm_name_);
			LOG_ERROR("failed to set size of shared memory object");
			return false;
		}

		shm_ptr_ = static_cast<unsigned char *>(mmap(0, size, PROT_WRITE, MAP_SHARED, shm_fd_, 0));
		if (shm_ptr_ == MAP_FAILED)
		{
			close(shm_fd_);
			shm_unlink(shm_name_);
			LOG_ERROR("failed to map shared memory object");
			return false;
		}

		sem_close(sem_read_);
		sem_unlink(sem_read_name_);
		sem_read_ = sem_open(sem_read_name_, O_CREAT, 0666, 0);
		if (sem_read_ == SEM_FAILED)
		{
			munmap(shm_ptr_, size);
			close(shm_fd_);
			shm_unlink(shm_name_);
			LOG_ERROR("failed to open sem_read semaphore");
			return false;
		}

		sem_close(sem_write_);
		sem_unlink(sem_write_name_);
		sem_write_ = sem_open(sem_write_name_, O_CREAT, 0666, 1);
		if (sem_write_ == SEM_FAILED)
		{
			sem_close(sem_read_);
			sem_unlink(sem_read_name_);
			munmap(shm_ptr_, size);
			close(shm_fd_);
			shm_unlink(shm_name_);
			LOG_ERROR("failed to open sem_write semaphore");
			return false;
		}

		init_shared_memory_ = true;
		memory_size_ = size;
		return true;
	}
}

static bool change_memory_size(size_t size)
{
	if (memory_size_ == size)
	{
		return true;
	}
	else
	{
		if (shm_ptr_ != nullptr)
		{
			munmap(shm_ptr_, memory_size_);
		}

		if (ftruncate(shm_fd_, size) == -1)
		{
			sem_close(sem_read_);
			sem_unlink(sem_read_name_);
			sem_close(sem_write_);
			sem_unlink(sem_write_name_);
			close(shm_fd_);
			shm_unlink(shm_name_);
			init_shared_memory_ = false;
			LOG_ERROR("failed to set size of shared memory object");
			return false;
		}

		shm_ptr_ = static_cast<unsigned char *>(mmap(NULL, size, PROT_WRITE, MAP_SHARED, shm_fd_, 0));
		if (shm_ptr_ == MAP_FAILED)
		{
			sem_close(sem_read_);
			sem_unlink(sem_read_name_);
			sem_close(sem_write_);
			sem_unlink(sem_write_name_);
			close(shm_fd_);
			shm_unlink(shm_name_);
			init_shared_memory_ = false;
			LOG_ERROR("failed to map shared memory object");
			return false;
		}
		memory_size_ = size;
		return true;
	}
}

static void yuyv_write(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
					   StillOptions const *options, std::vector<Detection> const &detections)
{
	if (options->encoding == "yuv420")
	{
		unsigned w = info.width, h = info.height, stride = info.stride;
		if ((w & 1) || (h & 1))
			throw std::runtime_error("both width and height must be even");

		size_t detections_size = sizeof(int);
		for (const auto &detection : detections)
        {
            detections_size += sizeof(int) + sizeof(float) + sizeof(int) * 4 + detection.name.size() + 1;
        }

		size_t shm_size = sizeof(int) * 2 + options->encoding.size() + 1 + mem[0].size() + detections_size;

		if (init_shared_memory(shm_size) && change_memory_size(shm_size))
		{
			int sem_write_value;
			sem_getvalue(sem_write_, &sem_write_value);
			if (sem_write_value == 1)
			{
				uint8_t *ptr = (uint8_t *)mem[0].data();

				int *shm_int_ptr = (int *)shm_ptr_;
				shm_int_ptr[0] = w;
				shm_int_ptr[1] = h;

				char *shm_char_ptr = (char *)(shm_int_ptr + 2);
				std::strcpy(shm_char_ptr, options->encoding.c_str());

				uint8_t *shm_image_ptr = (uint8_t *)(shm_char_ptr + options->encoding.size() + 1);

				std::vector<uint8_t> row(w);
				for (unsigned int j = 0; j < h; j++, ptr += stride)
				{
					for (unsigned int i = 0; i < w; i++)
						row[i] = ptr[i << 1];
					std::memcpy(shm_image_ptr + j * w, &row[0], w);
				}

				ptr = (uint8_t *)mem[0].data();
				shm_image_ptr += h * w;
				for (unsigned int j = 0; j < h; j += 2, ptr += 2 * stride)
				{
					for (unsigned int i = 0; i < w / 2; i++)
						row[i] = ptr[(i << 2) + 1];
					std::memcpy(shm_image_ptr + (j / 2) * (w / 2), &row[0], w / 2);
				}

				ptr = (uint8_t *)mem[0].data();
				shm_image_ptr += (h / 2) * (w / 2);
				for (unsigned int j = 0; j < h; j += 2, ptr += 2 * stride)
				{
					for (unsigned int i = 0; i < w / 2; i++)
						row[i] = ptr[(i << 2) + 3];
					std::memcpy(shm_image_ptr + (j / 2) * (w / 2), &row[0], w / 2);
				}

				uint8_t *shm_detections_ptr = shm_image_ptr + h * w;
				int *detections_count_ptr = (int *)shm_detections_ptr;
				*detections_count_ptr = detections.size();
				shm_detections_ptr += sizeof(int);

				for (const auto &detection : detections)
				{
					int *category_ptr = (int *)shm_detections_ptr;
					*category_ptr = detection.category;
					shm_detections_ptr += sizeof(int);

					float *confidence_ptr = (float *)shm_detections_ptr;
					*confidence_ptr = detection.confidence;
					shm_detections_ptr += sizeof(float);

					int *box_ptr = (int *)shm_detections_ptr;
					box_ptr[0] = detection.box.x;
					box_ptr[1] = detection.box.y;
					box_ptr[2] = detection.box.width;
					box_ptr[3] = detection.box.height;
					shm_detections_ptr += sizeof(int) * 4;

					char *name_ptr = (char *)shm_detections_ptr;
					std::strcpy(name_ptr, detection.name.c_str());
					shm_detections_ptr += detection.name.size() + 1;
				}

				sem_post(sem_read_);
				sem_trywait(sem_write_);
			}
		}
	}
	else
		throw std::runtime_error("output format " + options->encoding + " not supported");
}

static void yuv420_write(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
						 StillOptions const *options, std::vector<Detection> const &detections)
{
	if (options->encoding == "yuv420")
	{
		unsigned w = info.width, h = info.height, stride = info.stride;
		if ((w & 1) || (h & 1))
			throw std::runtime_error("both width and height must be even");
		if (mem.size() != 1)
			throw std::runtime_error("incorrect number of planes in YUV420 data");

		size_t detections_size = sizeof(int);
		for (const auto &detection : detections)
        {
            detections_size += sizeof(int) + sizeof(float) + sizeof(int) * 4 + detection.name.size() + 1;
        }

		size_t shm_size = sizeof(int) * 2 + options->encoding.size() + 1 + mem[0].size() + detections_size;

		if (init_shared_memory(shm_size) && change_memory_size(shm_size))
		{
			int sem_write_value;
			sem_getvalue(sem_write_, &sem_write_value);
			if (sem_write_value == 1)
			{
				uint8_t *Y = (uint8_t *)mem[0].data();

				int *shm_int_ptr = (int *)shm_ptr_;
				shm_int_ptr[0] = w;
				shm_int_ptr[1] = h;

				char *shm_char_ptr = (char *)(shm_int_ptr + 2);
				std::strcpy(shm_char_ptr, options->encoding.c_str());

				uint8_t *shm_image_ptr = (uint8_t *)(shm_char_ptr + options->encoding.size() + 1);
				for (unsigned int j = 0; j < h; j++)
				{
					std::memcpy(shm_image_ptr + j * w, Y + j * stride, w);
				}

				uint8_t *U = Y + stride * h;
				h /= 2, w /= 2, stride /= 2;
				shm_image_ptr += h * w;
				for (unsigned int j = 0; j < h; j++)
				{
					std::memcpy(shm_image_ptr + j * w, U + j * stride, w);
				}

				uint8_t *V = U + stride * h;
				shm_image_ptr += h * w;
				for (unsigned int j = 0; j < h; j++)
				{
					std::memcpy(shm_image_ptr + j * w, V + j * stride, w);
				}

				uint8_t *shm_detections_ptr = shm_image_ptr + h * w;
				int *detections_count_ptr = (int *)shm_detections_ptr;
				*detections_count_ptr = detections.size();
				shm_detections_ptr += sizeof(int);

				for (const auto &detection : detections)
				{
					int *category_ptr = (int *)shm_detections_ptr;
					*category_ptr = detection.category;
					shm_detections_ptr += sizeof(int);

					float *confidence_ptr = (float *)shm_detections_ptr;
					*confidence_ptr = detection.confidence;
					shm_detections_ptr += sizeof(float);

					int *box_ptr = (int *)shm_detections_ptr;
					box_ptr[0] = detection.box.x;
					box_ptr[1] = detection.box.y;
					box_ptr[2] = detection.box.width;
					box_ptr[3] = detection.box.height;
					shm_detections_ptr += sizeof(int) * 4;

					char *name_ptr = (char *)shm_detections_ptr;
					std::strcpy(name_ptr, detection.name.c_str());
					shm_detections_ptr += detection.name.size() + 1;
				}

				sem_post(sem_read_);
				sem_trywait(sem_write_);
			}
		}
	}
	else
		throw std::runtime_error("output format " + options->encoding + " not supported");
}

static void rgb_write(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
					  StillOptions const *options, std::vector<Detection> const &detections)
{
	if (options->encoding != "rgb24" && options->encoding != "rgb48")
		throw std::runtime_error("encoding should be set to rgb");

	size_t detections_size = sizeof(int);
	for (const auto &detection : detections)
	{
		detections_size += sizeof(int) + sizeof(float) + sizeof(int) * 4 + detection.name.size() + 1;
	}

	size_t shm_size = sizeof(int) * 2 + options->encoding.size() + 1 + mem[0].size() + detections_size;

	if (init_shared_memory(shm_size) && change_memory_size(shm_size))
	{
		unsigned w = info.width, h = info.height, stride = info.stride;
		int sem_write_value;
		sem_getvalue(sem_write_, &sem_write_value);
		if (sem_write_value == 1)
		{
			uint8_t *ptr = (uint8_t *)mem[0].data();
			unsigned int wr_stride = 3 * w;
			if (options->encoding == "rgb48")
				wr_stride *= 2;

			int *shm_int_ptr = (int *)shm_ptr_;
			shm_int_ptr[0] = w;
			shm_int_ptr[1] = h;

			char *shm_char_ptr = (char *)(shm_int_ptr + 2);
			std::strcpy(shm_char_ptr, options->encoding.c_str());

			uint8_t *shm_image_ptr = (uint8_t *)(shm_char_ptr + options->encoding.size() + 1);
			for (unsigned int j = 0; j < h; j++, ptr += stride)
			{
				std::memcpy(shm_image_ptr + j * wr_stride, ptr, wr_stride);
			}

			uint8_t *shm_detections_ptr = shm_image_ptr + h * w;
			int *detections_count_ptr = (int *)shm_detections_ptr;
			*detections_count_ptr = detections.size();
			shm_detections_ptr += sizeof(int);

			for (const auto &detection : detections)
			{
				int *category_ptr = (int *)shm_detections_ptr;
				*category_ptr = detection.category;
				shm_detections_ptr += sizeof(int);

				float *confidence_ptr = (float *)shm_detections_ptr;
				*confidence_ptr = detection.confidence;
				shm_detections_ptr += sizeof(float);

				int *box_ptr = (int *)shm_detections_ptr;
				box_ptr[0] = detection.box.x;
				box_ptr[1] = detection.box.y;
				box_ptr[2] = detection.box.width;
				box_ptr[3] = detection.box.height;
				shm_detections_ptr += sizeof(int) * 4;

				char *name_ptr = (char *)shm_detections_ptr;
				std::strcpy(name_ptr, detection.name.c_str());
				shm_detections_ptr += detection.name.size() + 1;
			}

			sem_post(sem_read_);
			sem_trywait(sem_write_);
		}
	}
}

static void write_images(RPiCamWriteApp &app, CompletedRequestPtr &payload, Stream *stream, StillOptions const *options)
{
	BufferReadSync r(&app, payload->buffers[stream]);
	const std::vector<libcamera::Span<uint8_t>> mem = r.Get();
	StreamInfo info = app.GetStreamInfo(stream);

	std::vector<Detection> detections;
	payload->post_process_metadata.Get("object_detect.results", detections);

	if (info.pixel_format == libcamera::formats::YUYV)
		yuyv_write(mem, info, options, detections);
	else if (info.pixel_format == libcamera::formats::YUV420)
		yuv420_write(mem, info, options, detections);
	else if (info.pixel_format == libcamera::formats::BGR888 || info.pixel_format == libcamera::formats::RGB888 ||
			 info.pixel_format == libcamera::formats::BGR161616 || info.pixel_format == libcamera::formats::RGB161616)
		rgb_write(mem, info, options, detections);
	else
		throw std::runtime_error("unrecognised YUV/RGB save format");
}

static void event_loop(RPiCamWriteApp &app)
{
	StillOptions *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureStill();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0;; count++)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			return;
		else if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		LOG(2, "Still frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->timeout && (now - start_time) > options->timeout.value)
			return;

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		write_images(app, completed_request, app.StillStream(), options);
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamWriteApp app;
		StillOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
