/*
 *  Copyright (C) Texas Instruments Incorporated 2026
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  tsndemo_stats.c
 *
 * \brief TSN demo talker - latency statistics and report printing.
 *
 *        Fixed-bin histograms: O(1) memory, O(1) per sample, no allocation,
 *        no sorting. Percentiles are computed by walking the bins; jitter
 *        headline is PDV = p99 - min.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <string.h>
#include "tsndemo_talker.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void TsnDemoStats_histReset(TsnDemo_LatencyHist *hist)
{
    memset(hist, 0, sizeof(*hist));
    hist->minNs = 0xFFFFFFFFU;
}

void TsnDemoStats_reset(TsnDemo_RunStats *stats, TsnDemo_RunMode mode,
                        const TsnDemo_FeatureSet *features)
{
    memset(stats, 0, sizeof(*stats));
    TsnDemoStats_histReset(&stats->fwd);
    TsnDemoStats_histReset(&stats->rev);
    TsnDemoStats_histReset(&stats->rtt);
    TsnDemoStats_histReset(&stats->turn);
    stats->mode     = mode;
    stats->features = *features;
    stats->valid    = true;
}

void TsnDemoStats_addSample(TsnDemo_LatencyHist *hist, uint64_t tEarlyNs,
                            uint64_t tLateNs)
{
    uint64_t deltaNs;
    uint32_t d32;

    /* a timestamp read failure (either side == 0) is not countable */
    if ((tEarlyNs != 0ULL) && (tLateNs != 0ULL))
    {
        if (tLateNs < tEarlyNs)
        {
            hist->negativeCount++;
        }
        else
        {
            deltaNs = tLateNs - tEarlyNs;

            hist->count++;
            hist->sumNs += deltaNs;
            /* sumSq in us^2-scale to keep 64-bit safe over long runs */
            hist->sumSqNs += (deltaNs / 100ULL) * (deltaNs / 100ULL);

            d32 = (deltaNs > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)deltaNs;
            if (d32 < hist->minNs)
            {
                hist->minNs = d32;
            }
            if (d32 > hist->maxNs)
            {
                hist->maxNs = d32;
            }

            if (deltaNs >= ((uint64_t)TSNDEMO_HIST_BINS * TSNDEMO_HIST_BIN_NS))
            {
                hist->overflowCount++;
            }
            else
            {
                hist->bins[deltaNs / TSNDEMO_HIST_BIN_NS]++;
            }
        }
    }
}

uint32_t TsnDemoStats_percentile(const TsnDemo_LatencyHist *hist,
                                 uint32_t pctX100)
{
    uint64_t inBinCount = hist->count - hist->overflowCount;
    uint64_t target;
    uint64_t acc = 0ULL;
    uint32_t result = 0U;
    uint32_t i;

    if (hist->count != 0U)
    {
        target = (hist->count * pctX100 + 9999ULL) / 10000ULL;
        if (target > inBinCount)
        {
            /* percentile falls into the overflow region */
            result = hist->maxNs;
        }
        else
        {
            result = hist->maxNs;
            for (i = 0U; (i < TSNDEMO_HIST_BINS) && (acc < target); i++)
            {
                acc += hist->bins[i];
                if (acc >= target)
                {
                    /* upper edge of the bin */
                    result = (i + 1U) * TSNDEMO_HIST_BIN_NS;
                }
            }
        }
    }
    return result;
}

#if 0
/* Unused while the PDV fwd line in TsnDemoStats_print() is commented out
 * (see docs/networking/enet_lld/tsn_demo_percentile_stats.md) - isqrt's
 * only caller is stddevNs, whose only caller is that line. Restore both
 * alongside it. */

/*
 * Integer square root (Newton-Raphson). Avoids pulling libm/sqrt() into the
 * link for a display-only value.
 */
static uint64_t TsnDemoStats_isqrt(uint64_t value)
{
    uint64_t root = value;
    uint64_t prev = 0ULL;

    if (value != 0ULL)
    {
        /* converges quickly; the equality guard also stops 2-cycle oscillation */
        while ((root != prev) && (root != (prev + 1ULL)))
        {
            prev = root;
            root = (root + (value / root)) / 2ULL;
        }
    }
    return root;
}

static uint32_t TsnDemoStats_stddevNs(const TsnDemo_LatencyHist *hist)
{
    uint64_t mean100, meanSq100, var100;
    uint32_t result = 0U;

    if (hist->count >= 2U)
    {
        /* sumSqNs accumulates (delta/100)^2, so work in (100 ns) units */
        mean100   = (hist->sumNs / hist->count) / 100ULL;
        meanSq100 = hist->sumSqNs / hist->count;
        if (meanSq100 >= (mean100 * mean100))
        {
            var100 = meanSq100 - (mean100 * mean100);
            result = (uint32_t)(TsnDemoStats_isqrt(var100) * 100ULL);
        }
        /* else: rounding underflow on a near-constant distribution -> 0 */
    }
    return result;
}
#endif

static void TsnDemoStats_printHistLine(const char *name,
                                       const TsnDemo_LatencyHist *hist)
{
    if (hist->count == 0U)
    {
        EnetAppUtils_print(" %-24s : no samples\r\n", name);
    }
    else
    {
        EnetAppUtils_print(
            " %-24s min %7u  p50 %7u  p95 %7u  p99 %7u  max %7u ns\r\n",
            name, hist->minNs,
            TsnDemoStats_percentile(hist, 5000U),
            TsnDemoStats_percentile(hist, 9500U),
            TsnDemoStats_percentile(hist, 9900U),
            hist->maxNs);
        if (hist->overflowCount != 0U)
        {
            EnetAppUtils_print("   WARNING: %u samples beyond histogram range "
                               "(%u ns) - percentiles above are clipped\r\n",
                               hist->overflowCount,
                               TSNDEMO_HIST_BINS * TSNDEMO_HIST_BIN_NS);
        }
        if (hist->negativeCount != 0U)
        {
            EnetAppUtils_print("   WARNING: %u negative deltas discarded "
                               "(clock not stable?)\r\n", hist->negativeCount);
        }
    }
}

/*
 * Simplified min/avg/max print for RTT, used instead of the percentile
 * breakdown in TsnDemoStats_printHistLine() while listener-side T2/T3
 * stamping is disabled (see TsnDemoListener_rxFrameCb and
 * docs/networking/enet_lld/tsn_demo_percentile_stats.md).
 */
static void TsnDemoStats_printRtt(const char *name,
                                  const TsnDemo_LatencyHist *hist)
{
    uint32_t avgNs;

    if (hist->count == 0U)
    {
        EnetAppUtils_print(" %-24s : no samples\r\n", name);
    }
    else
    {
        avgNs = (uint32_t)(hist->sumNs / hist->count);
        EnetAppUtils_print(" %-24s min %7u  avg %7u  max %7u ns\r\n",
                           name, hist->minNs, avgNs, hist->maxNs);
        if (hist->overflowCount != 0U)
        {
            EnetAppUtils_print("   WARNING: %u samples beyond histogram range "
                               "(%u ns)\r\n", hist->overflowCount,
                               TSNDEMO_HIST_BINS * TSNDEMO_HIST_BIN_NS);
        }
        if (hist->negativeCount != 0U)
        {
            EnetAppUtils_print("   WARNING: %u negative deltas discarded "
                               "(clock not stable?)\r\n", hist->negativeCount);
        }
    }
}

static const char *TsnDemoStats_modeName(TsnDemo_RunMode mode)
{
    const char *name;

    switch (mode)
    {
        case TSNDEMO_MODE_BASELINE:         name = "BASELINE";           break;
        case TSNDEMO_MODE_CONTENTION_NOTSN: name = "CONTENTION-TSN-OFF"; break;
        case TSNDEMO_MODE_CONTENTION_TSN:   name = "CONTENTION-TSN-ON";  break;
        default:                            name = "?";                 break;
    }
    return name;
}

void TsnDemoStats_print(const TsnDemo_RunStats *stats)
{
    uint64_t windowNs = (stats->windowEndNs > stats->windowStartNs)
                        ? (stats->windowEndNs - stats->windowStartNs) : 0ULL;

    EnetAppUtils_print("\r\n=====================================================================\r\n");
    EnetAppUtils_print(" Run report | mode %s | EST=%s IET=%s CUTTHRU=%s%s\r\n",
                       TsnDemoStats_modeName(stats->mode),
                       stats->features.estEn ? "on" : "off",
                       stats->features.ietEn ? "on" : "off",
                       stats->features.cutThruEn ? "on" : "off",
                       stats->valid ? "" : "  ** INVALID **");
    EnetAppUtils_print(" window %u.%03u s, %u samples\r\n",
                       (uint32_t)(windowNs / 1000000000ULL),
                       (uint32_t)((windowNs % 1000000000ULL) / 1000000ULL),
                       stats->rxCount);
    EnetAppUtils_print("---------------------------------------------------------------------\r\n");
    TsnDemoStats_printHistLine("express one-way (T2-T1)", &stats->fwd);
//    TsnDemoStats_printHistLine("echo one-way    (T4-T3)", &stats->rev);
    TsnDemoStats_printRtt("round trip      (T4-T1)", &stats->rtt);
//    TsnDemoStats_printHistLine("turnaround      (T3-T2)", &stats->turn);
//    EnetAppUtils_print(" PDV fwd (p99-min) %u ns   max-min %u ns   sd %u ns\r\n",
//                       TsnDemoStats_percentile(&stats->fwd, 9900U) -
//                       ((stats->fwd.count != 0U) ? stats->fwd.minNs : 0U),
//                       (stats->fwd.count != 0U) ? (stats->fwd.maxNs - stats->fwd.minNs) : 0U,
//                       TsnDemoStats_stddevNs(&stats->fwd));
    EnetAppUtils_print("---------------------------------------------------------------------\r\n");
    EnetAppUtils_print(" tx %u  rx %u  lost %u (%u.%03u%%)  ooo %u  stale %u  ts-fail %u\r\n",
                       stats->txCount, stats->rxCount, stats->lostCount,
                       (stats->txCount != 0U) ?
                           (stats->lostCount * 100U) / stats->txCount : 0U,
                       (stats->txCount != 0U) ?
                           ((uint32_t)(((uint64_t)stats->lostCount * 100000ULL) /
                                       stats->txCount)) % 1000U : 0U,
                       stats->outOfOrderCount, stats->staleCount, stats->tsFailCount);
    if ((stats->bulkTxBytes != 0ULL) && (windowNs != 0ULL))
    {
        uint32_t mbps = (uint32_t)((stats->bulkTxBytes * 8000ULL) / windowNs);
        EnetAppUtils_print(" bulk offered load: %u Mbit/s\r\n", mbps);
    }
    EnetAppUtils_print("=====================================================================\r\n");
}

static void TsnDemoStats_printCompareRow(const char *name,
                                         uint32_t offNs, uint32_t onNs)
{
    uint32_t ratioX10 = (onNs != 0U) ? ((offNs * 10U) / onNs) : 0U;

    EnetAppUtils_print(" %-24s %9u ns  %9u ns  %3u.%ux\r\n",
                       name, offNs, onNs, ratioX10 / 10U, ratioX10 % 10U);
}

void TsnDemoStats_printComparison(const TsnDemo_RunStats *tsnOff,
                                  const TsnDemo_RunStats *tsnOn)
{
    uint64_t winOff;
    uint64_t winOn;
    uint32_t mbpsOff;
    uint32_t mbpsOn;

    if ((tsnOff->rxCount == 0U) || (tsnOn->rxCount == 0U))
    {
        EnetAppUtils_print("Comparison needs one completed TSN-off run and "
                           "one TSN-on run.\r\n");
    }
    else
    {
        EnetAppUtils_print("\r\n============ HEADLINE: same contention, TSN off vs on ==============\r\n");
        EnetAppUtils_print(" %-24s %12s  %12s  improvement\r\n", "",
                           "TSN OFF", "TSN ON");
        TsnDemoStats_printCompareRow("express p99 one-way",
                                     TsnDemoStats_percentile(&tsnOff->fwd, 9900U),
                                     TsnDemoStats_percentile(&tsnOn->fwd, 9900U));
        TsnDemoStats_printCompareRow("express PDV (p99-min)",
                                     TsnDemoStats_percentile(&tsnOff->fwd, 9900U) - tsnOff->fwd.minNs,
                                     TsnDemoStats_percentile(&tsnOn->fwd, 9900U) - tsnOn->fwd.minNs);
        TsnDemoStats_printCompareRow("express max",
                                     tsnOff->fwd.maxNs, tsnOn->fwd.maxNs);
        EnetAppUtils_print(" %-24s %11u.%03u%%  %10u.%03u%%\r\n", "express loss",
                           (tsnOff->txCount != 0U) ? (tsnOff->lostCount * 100U) / tsnOff->txCount : 0U,
                           (tsnOff->txCount != 0U) ?
                               ((uint32_t)(((uint64_t)tsnOff->lostCount * 100000ULL) / tsnOff->txCount)) % 1000U : 0U,
                           (tsnOn->txCount != 0U) ? (tsnOn->lostCount * 100U) / tsnOn->txCount : 0U,
                           (tsnOn->txCount != 0U) ?
                               ((uint32_t)(((uint64_t)tsnOn->lostCount * 100000ULL) / tsnOn->txCount)) % 1000U : 0U);

        winOff = tsnOff->windowEndNs - tsnOff->windowStartNs;
        winOn  = tsnOn->windowEndNs - tsnOn->windowStartNs;
        if ((winOff != 0ULL) && (winOn != 0ULL) && (tsnOff->bulkTxBytes != 0ULL))
        {
            mbpsOff = (uint32_t)((tsnOff->bulkTxBytes * 8000ULL) / winOff);
            mbpsOn  = (uint32_t)((tsnOn->bulkTxBytes * 8000ULL) / winOn);

            EnetAppUtils_print(" %-24s %8u Mbit/s  %8u Mbit/s  (cost of TSN)\r\n",
                               "bulk throughput", mbpsOff, mbpsOn);
        }
        EnetAppUtils_print("=====================================================================\r\n");
    }
}
