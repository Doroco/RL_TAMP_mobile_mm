/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Julian Seward, Nikolaus Vahrenkamp
* @copyright  2011 Julian Seward, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*
* The BZIP2 compression/decompression routines are based on the bzip2 library:
   bzip2/libbzip2 version 1.0.6 of 6 September 2010
   Copyright (C) 1996-2010 Julian Seward <jseward@bzip.org>
*/

#pragma once

#include "Exception.h"
#include "RobotWorkSpace.h"
#define BZ_RAND_DECLS                          \
    Int32 rNToGo;                               \
    Int32 rTPos                                 \


#define BZ_MAX_ALPHA_SIZE 258
#define BZ_MAX_CODE_LEN    23

#define BZ_MAX_UNUSED 5000

#define BZ_RUNA 0
#define BZ_RUNB 1

#define BZ_N_GROUPS 6
#define BZ_G_SIZE   50
#define BZ_N_ITERS  4

#define BZ_MAX_SELECTORS (2 + (900000 / BZ_G_SIZE))


#define MTFA_SIZE 4096
#define MTFL_SIZE 16

#include <fstream>

namespace RobotWorkSpace
{

    class CompressionBZip2
    {
    public:

        enum CompressionMode
        {
            eCompress,
            eUncompress
        };

        /*!
            Initialize file for storing output.
            Overwrites the file if already existing.
        */
        //CompressionBZip2(const std::string &filename, CompressionMode mode);
        CompressionBZip2(std::istream* ifs);// , CompressionMode mode); // read mode
        CompressionBZip2(std::ostream* ofs);// , CompressionMode mode); // write mode
        virtual ~CompressionBZip2();


        /*!
            Close file and reset data. No further operations are possible.
            Be sure to call this before output-stream is deleted (only in eCompress mode)! Maybe there is some data in the internal buffers which must be flushed.
        */
        bool close();

        /*!
            Only allowed in eCompress mode.
            Compress buf with given length and stores data to file.
        */
        bool write(void* buf, int len);

        /*!
            Only allowed in eUncompress mode.
            Read block from file. buf must be at least of size maxLen.
            storeLengthRead holds number of bytes that have been read (e.g. in case an EOF occurs).
        */
        bool read(void* buf, int maxLen, int& storeLengthRead);

    protected:
        /*!
            Returns read bytes.
        */
        int read(char* res, int num, std::istream* file);
        bool write(std::ostream* file, const char* value, int num);

        typedef char            Char;
        typedef unsigned char   Bool;
        typedef unsigned char   UChar;
        typedef int             Int32;
        typedef unsigned int    UInt32;
        typedef short           Int16;
        typedef unsigned short  UInt16;

        typedef void BZFILE;

        typedef
        struct
        {
            char* next_in;
            unsigned int avail_in;
            unsigned int total_in_lo32;
            unsigned int total_in_hi32;

            char* next_out;
            unsigned int avail_out;
            unsigned int total_out_lo32;
            unsigned int total_out_hi32;

            void* state;

            void* (*bzalloc)(void*, int, int);
            void (*bzfree)(void*, void*);
            void* opaque;
        }
        bz_stream;
        typedef
        struct
        {
            /* pointer back to the struct bz_stream */
            bz_stream* strm;

            /* mode this stream is in, and whether inputting */
            /* or outputting data */
            Int32    mode;
            Int32    state;

            /* remembers avail_in when flush/finish requested */
            UInt32   avail_in_expect;

            /* for doing the block sorting */
            UInt32*  arr1;
            UInt32*  arr2;
            UInt32*  ftab;
            Int32    origPtr;

            /* aliases for arr1 and arr2 */
            UInt32*  ptr;
            UChar*   block;
            UInt16*  mtfv;
            UChar*   zbits;

            /* for deciding when to use the fallback sorting algorithm */
            Int32    workFactor;

            /* run-length-encoding of the input */
            UInt32   state_in_ch;
            Int32    state_in_len;
            BZ_RAND_DECLS;
            /* input and output limits and current posns */
            Int32    nblock;
            Int32    nblockMAX;
            Int32    numZ;
            Int32    state_out_pos;

            /* map of bytes used in block */
            Int32    nInUse;
            Bool     inUse[256];
            UChar    unseqToSeq[256];

            /* the buffer for bit stream creation */
            UInt32   bsBuff;
            Int32    bsLive;

            /* block and combined CRCs */
            UInt32   blockCRC;
            UInt32   combinedCRC;

            /* misc administratium */
            Int32    verbosity;
            Int32    blockNo;
            Int32    blockSize100k;

            /* stuff for coding the MTF values */
            Int32    nMTF;
            Int32    mtfFreq    [BZ_MAX_ALPHA_SIZE];
            UChar    selector   [BZ_MAX_SELECTORS];
            UChar    selectorMtf[BZ_MAX_SELECTORS];

            UChar    len     [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            Int32    code    [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            Int32    rfreq   [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            /* second dimension: only 3 needed; 4 makes index calculations faster */
            UInt32   len_pack[BZ_MAX_ALPHA_SIZE][4];

        }
        EState;

        typedef
        struct
        {
            /* pointer back to the struct bz_stream */
            bz_stream* strm;

            /* state indicator for this stream */
            Int32    state;

            /* for doing the final run-length decoding */
            UChar    state_out_ch;
            Int32    state_out_len;
            Bool     blockRandomised;
            BZ_RAND_DECLS;

            /* the buffer for bit stream reading */
            UInt32   bsBuff;
            Int32    bsLive;

            /* misc administratium */
            Int32    blockSize100k;
            Bool     smallDecompress;
            Int32    currBlockNo;
            Int32    verbosity;

            /* for undoing the Burrows-Wheeler transform */
            Int32    origPtr;
            UInt32   tPos;
            Int32    k0;
            Int32    unzftab[256];
            Int32    nblock_used;
            Int32    cftab[257];
            Int32    cftabCopy[257];

            /* for undoing the Burrows-Wheeler transform (FAST) */
            UInt32*   tt;

            /* for undoing the Burrows-Wheeler transform (SMALL) */
            UInt16*   ll16;
            UChar*    ll4;

            /* stored and calculated CRCs */
            UInt32   storedBlockCRC;
            UInt32   storedCombinedCRC;
            UInt32   calculatedBlockCRC;
            UInt32   calculatedCombinedCRC;

            /* map of bytes used in block */
            Int32    nInUse;
            Bool     inUse[256];
            Bool     inUse16[16];
            UChar    seqToUnseq[256];

            /* for decoding the MTF values */
            UChar    mtfa   [MTFA_SIZE];
            Int32    mtfbase[256 / MTFL_SIZE];
            UChar    selector   [BZ_MAX_SELECTORS];
            UChar    selectorMtf[BZ_MAX_SELECTORS];
            UChar    len  [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];

            Int32    limit  [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            Int32    base   [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            Int32    perm   [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
            Int32    minLens[BZ_N_GROUPS];

            /* save area for scalars in the main decompress code */
            Int32    save_i;
            Int32    save_j;
            Int32    save_t;
            Int32    save_alphaSize;
            Int32    save_nGroups;
            Int32    save_nSelectors;
            Int32    save_EOB;
            Int32    save_groupNo;
            Int32    save_groupPos;
            Int32    save_nextSym;
            Int32    save_nblockMAX;
            Int32    save_nblock;
            Int32    save_es;
            Int32    save_N;
            Int32    save_curr;
            Int32    save_zt;
            Int32    save_zn;
            Int32    save_zvec;
            Int32    save_zj;
            Int32    save_gSel;
            Int32    save_gMinlen;
            Int32*   save_gLimit;
            Int32*   save_gBase;
            Int32*   save_gPerm;

        }
        DState;



        static void BZ2_bsInitWrite(EState* s);
        Int32 BZ2_decompress(DState* s);
        static void BZ2_compressBlock(EState* s, Bool is_last_block);

        static CompressionBZip2::Bool myfeof(std::istream* f);  // FILE* f );

        static int bz_config_ok(void);

        static void add_pair_to_block(EState* s);
        static Bool copy_input_until_stop(EState* s);
        static Bool copy_output_until_stop(EState* s);

        static Bool unRLE_obuf_to_output_SMALL(DState* s);
        static Bool unRLE_obuf_to_output_FAST(DState* s);



        int BZ2_bzDecompress(bz_stream* strm);
        int BZ2_bzDecompressEnd(bz_stream* strm);

        int BZ2_bzCompressEnd(bz_stream* strm);

        BZFILE* BZ2_bzWriteOpen
        (int*  bzerror,
         std::ostream* f,
         //FILE* f,
         int   blockSize100k = 9,
         int   verbosity = 0,
         int   workFactor = 30);

        void BZ2_bzWrite
        (int*    bzerror,
         BZFILE* b,
         void*   buf,
         int     len);

        void BZ2_bzWriteClose
        (int*          bzerror,
         BZFILE*       b,
         int           abandon,
         unsigned int* nbytes_in,
         unsigned int* nbytes_out);

        void BZ2_bzWriteClose64
        (int*          bzerror,
         BZFILE*       b,
         int           abandon,
         unsigned int* nbytes_in_lo32,
         unsigned int* nbytes_in_hi32,
         unsigned int* nbytes_out_lo32,
         unsigned int* nbytes_out_hi32);

        BZFILE* BZ2_bzReadOpen
        (int*  bzerror,
         std::istream* f,
         //FILE* f,
         int   verbosity,
         int   smallValue,
         void* unused,
         int   nUnused);
        void BZ2_bzReadClose(int* bzerror, BZFILE* b);
        int BZ2_bzRead
        (int*    bzerror,
         BZFILE* b,
         void*   buf,
         int     len);
        int BZ2_bzDecompressInit
        (bz_stream* strm,
         int        verbosity,
         int        smallValue);

        typedef
        struct
        {
            //FILE*     handle;
            std::istream*   handleIn;
            std::ostream*   handleOut;
            Char      buf[BZ_MAX_UNUSED];
            Int32     bufN;
            Bool      writing;
            bz_stream strm;
            Int32     lastErr;
            Bool      initialisedOk;
        }
        bzFile;


        static void bsFinishWrite(EState* s);
        static void bsW(EState* s, Int32 n, UInt32 v);

        static void bsPutUInt32(EState* s, UInt32 u);
        static void bsPutUChar(EState* s, UChar c);
        static void makeMaps_e(EState* s);

        static void generateMTFValues(EState* s);
        static void sendMTFValues(EState* s);
        static void makeMaps_d(DState* s);
        static void* default_bzalloc(void* opaque, Int32 items, Int32 size);
        static void default_bzfree(void* opaque, void* addr);
        static void init_RL(EState* s);
        static void prepare_new_block(EState* s);
        static Bool isempty_RL(EState* s);
        static void flush_RL(EState* s);

        static Bool handle_compress(bz_stream* strm);

        int BZ2_bzCompress(bz_stream* strm, int action);
        static Int32 BZ2_indexIntoF(Int32 indx, Int32* cftab);
        int BZ2_bzCompressInit
        (bz_stream* strm,
         int        blockSize100k,
         int        verbosity,
         int        workFactor);

        static void BZ2_hbMakeCodeLengths(UChar* len,
                                          Int32* freq,
                                          Int32 alphaSize,
                                          Int32 maxLen);
        static void BZ2_hbAssignCodes(Int32* code,
                                      UChar* length,
                                      Int32 minLen,
                                      Int32 maxLen,
                                      Int32 alphaSize);

        static void BZ2_hbCreateDecodeTables(Int32* limit,
                                             Int32* base,
                                             Int32* perm,
                                             UChar* length,
                                             Int32 minLen,
                                             Int32 maxLen,
                                             Int32 alphaSize);

        static void BZ2_blockSort(CompressionBZip2::EState* s);

        static void fallbackSort(UInt32* fmap,
                                 UInt32* eclass,
                                 UInt32* bhtab,
                                 Int32   nblock,
                                 Int32   verb);

        static void fallbackSimpleSort(UInt32* fmap,
                                       UInt32* eclass,
                                       Int32   lo,
                                       Int32   hi);
        static UChar mmed3(UChar a, UChar b, UChar c);
        static void mainQSort3(UInt32* ptr,
                               UChar*  block,
                               UInt16* quadrant,
                               Int32   nblock,
                               Int32   loSt,
                               Int32   hiSt,
                               Int32   dSt,
                               Int32*  budget);

        static void mainSimpleSort(UInt32* ptr,
                                   UChar*  block,
                                   UInt16* quadrant,
                                   Int32   nblock,
                                   Int32   lo,
                                   Int32   hi,
                                   Int32   d,
                                   Int32*  budget);

        static Bool mainGtU(UInt32  i1,
                            UInt32  i2,
                            UChar*  block,
                            UInt16* quadrant,
                            UInt32  nblock,
                            Int32*  budget);

        static Int32 incs[14];

        static void mainSort(UInt32* ptr,
                             UChar*  block,
                             UInt16* quadrant,
                             UInt32* ftab,
                             Int32   nblock,
                             Int32   verb,
                             Int32*  budget);

        static void fallbackQSort3(UInt32* fmap,
                                   UInt32* eclass,
                                   Int32   loSt,
                                   Int32   hiSt);



        static const UInt32 BZ2_crc32Table[256];
        static const Int32 BZ2_rNums[512];
        CompressionMode mode;
        //FILE *dataFile;
        BZFILE* bzFileData;
        int currentError;

        std::istream* ifs;
        std::ostream* ofs;

    };

    // typedef std::shared_ptr<CompressionBZip2> CompressionBZip2Ptr;

} // namespace

