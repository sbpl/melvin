#ifndef _FILETRANSFER_JMB
#define _FILETRANSFER_JMB
void getfiles(unsigned char **cover_map, unsigned char **cost_map, int16_t **elev_map, const char * filename, int &mapm, int &mapn);
void writefiles(const unsigned char cover_map[], const unsigned char cost_map[], const int16_t elev_map[], const char * filename, const int mapx, const int mapy);
void writefileextra(const int map[], const char * filename, int x, int y);
void writefiletraj( const double score, const std::vector<Traj_pt_s> & traj, const char * filename);
void writeBMP(const unsigned char cover_map[], const unsigned char cost_map[], const int mapm, const int mapn, const std::vector<Traj_pt_s> & traj, const char * filename);

typedef struct BMPType {
	unsigned char bfType[2];
} BMPType;

typedef struct BMPFileHdr {
		uint32_t bfSize;
		uint16_t bfres1;
		uint16_t bfres2;
		uint32_t bfOffset;
} BMPFileHeader;

typedef struct BMPInfoHdr {
	uint32_t biSize;
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes;
	uint16_t biBitCount;
	uint32_t biCompression;
	uint32_t biSizeImage;
	uint32_t biXPelsperMeter;
	uint32_t biYPelsperMeter;
	uint32_t biClrUsed;
	uint32_t biClrImportant;
} BMPInfoHeader;









#endif
