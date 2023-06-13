// ----------------------------------------------------------------------------
// -                        PCD RW                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 JD
// ----------------------------------------------------------------------------

#pragma once

namespace pcd
{
    /** \brief Compress in_len bytes stored at the memory block starting at
     * \a in_data and write the result to \a out_data, up to a maximum length
     * of \a out_len bytes using Marc Lehmann's LZF algorithm.
     *
     * If the output buffer is not large enough or any error occurs return 0,
     * otherwise return the number of bytes used, which might be considerably
     * more than in_len (but less than 104% of the original size), so it
     * makes sense to always use out_len == in_len - 1), to ensure _some_
     * compression, and store the data uncompressed otherwise (with a flag, of
     * course.
     *
     * \note The buffers must not be overlapping.
     *
     * \param[in] in_data the input uncompressed buffer
     * \param[in] in_len the length of the input buffer
     * \param[out] out_data the output buffer where the compressed result will be stored
     * \param[in] out_len the length of the output buffer
     *
     */
    unsigned int
    lzfCompress(const void *const in_data, unsigned int in_len,
                void *out_data, unsigned int out_len);

    /** \brief Decompress data compressed with the \a lzfCompress function and
     * stored at location \a in_data and length \a in_len. The result will be
     * stored at \a out_data up to a maximum of \a out_len characters.
     *
     * If the output buffer is not large enough to hold the decompressed
     * data, a 0 is returned and errno is set to E2BIG. Otherwise the number
     * of decompressed bytes (i.e. the original length of the data) is
     * returned.
     *
     * If an error in the compressed data is detected, a zero is returned and
     * errno is set to EINVAL.
     *
     * This function is very fast, about as fast as a copying loop.
     * \param[in] in_data the input compressed buffer
     * \param[in] in_len the length of the input buffer
     * \param[out] out_data the output buffer (must be resized to \a out_len)
     * \param[in] out_len the length of the output buffer
     */
    unsigned int
    lzfDecompress(const void *const in_data, unsigned int in_len,
                  void *out_data, unsigned int out_len);
}
