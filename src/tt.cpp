/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2022 The Stockfish developers (see AUTHORS file)

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstring>   // For std::memset
#include <iostream>
#include <thread>

#include "bitboard.h"
#include "misc.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"

namespace Stockfish {

TranspositionTable TT; // Our global transposition table

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, Color c) {

  uint32_t mask16 = c == WHITE ? ~0xFFFF : 0xFFFF;
  uint16_t mask8  = c == WHITE ? ~0xFF   : 0xFF;
  uint32_t key    = (key16 & 0x7FFFFFFF);
    // Preserve any existing move for the same position
  if (m || (k & 0x7FFFFFFF) != key)
      move16 = (move16 & ~mask16) | (m & mask16);

  // Overwrite less valuable entries (cheapest checks first)
  if (   b == BOUND_EXACT
      || (k & 0x7FFFFFFF) != key
      || d - DEPTH_OFFSET + 2 * pv > (depth8 & mask8) - 4)
  {
      assert(d > DEPTH_OFFSET);
      assert(d < 256 + DEPTH_OFFSET);

      key16     = (k & 0x7FFFFFFF) | ((c == WHITE ? 1 : 0) << 31);
      depth8    = (uint16_t)(((depth8 & ~mask8) | (d & mask8)) - DEPTH_OFFSET);
      genBound8 = (uint16_t)((genBound8 & ~mask8) | ((TT.generation8 | uint8_t(pv) << 2 | b) & mask8));
      value16   = c == WHITE ? make_valuett(v, value16 & ~mask16) : make_valuett(value16 & ~mask16, v);
      eval16    = c == WHITE ? make_valuett(ev, eval16 & ~mask16) : make_valuett(eval16 & ~mask16, ev);
  }
}


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resize(size_t mbSize) {

  Threads.main()->wait_for_search_finished();

  aligned_large_pages_free(table);

  clusterCount = mbSize * 1024 * 1024 / sizeof(Cluster);

  table = static_cast<Cluster*>(aligned_large_pages_alloc(clusterCount * sizeof(Cluster)));
  if (!table)
  {
      std::cerr << "Failed to allocate " << mbSize
                << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
  }

  clear();
}


/// TranspositionTable::clear() initializes the entire transposition table to zero,
//  in a multi-threaded way.

void TranspositionTable::clear() {

  std::vector<std::thread> threads;

  for (size_t idx = 0; idx < Options["Threads"]; ++idx)
  {
      threads.emplace_back([this, idx]() {

          // Thread binding gives faster search on systems with a first-touch policy
          if (Options["Threads"] > 8)
              WinProcGroup::bindThisThread(idx);

          // Each thread will zero its part of the hash table
          const size_t stride = size_t(clusterCount / Options["Threads"]),
                       start  = size_t(stride * idx),
                       len    = idx != Options["Threads"] - 1 ?
                                stride : clusterCount - start;

          std::memset(&table[start], 0, len * sizeof(Cluster));
      });
  }

  for (std::thread& th : threads)
      th.join();
}


/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.

TTEntry* TranspositionTable::probe(const Key key, bool& found, Color c) const {

  uint16_t mask8 = c == WHITE ? 0xFF00 : 0xFF;
  TTEntry* const tte = first_entry(key);
  const uint16_t key16 = key & 0x7FFFFFFF;  // Use the low 30 bits as key inside the cluster

  for (int i = 0; i < ClusterSize; ++i)
      if (tte[i].key16 == key16 || !tte[i].depth())
      {
          tte[i].genBound8 =(tte[i].genBound8 & ~mask8) | (generation8 | ((tte[i].genBound8 & mask8) & (GENERATION_DELTA - 1))); // Refresh

          return found = (bool)tte[i].depth(), &tte[i];
      }

  // Find an entry to be replaced according to the replacement strategy
  TTEntry* replace = tte;
  for (int i = 1; i < ClusterSize; ++i)
      // Due to our packed storage format for generation and its cyclic
      // nature we add GENERATION_CYCLE (256 is the modulus, plus what
      // is needed to keep the unrelated lowest n bits from affecting
      // the result) to calculate the entry age correctly even after
      // generation8 overflows into the next cycle.
      if (  replace->depth8 - ((GENERATION_CYCLE + generation8 - (replace->genBound8 & mask8)) & GENERATION_MASK)
          >   tte[i].depth8 - ((GENERATION_CYCLE + generation8 -   (tte[i].genBound8 & mask8)) & GENERATION_MASK))
          replace = &tte[i];

  return found = false, replace;
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000; ++i)
      for (int j = 0; j < ClusterSize; ++j)
          cnt += table[i].entry[j].depth8 && (table[i].entry[j].genBound8 & GENERATION_MASK) == generation8;

  return cnt / ClusterSize;
}

} // namespace Stockfish
