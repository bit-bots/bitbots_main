#include <gtest/gtest.h>

#include "bitbots_vision/candidate.hpp"

using bitbots_vision::Candidate;

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

TEST(Candidate, FromX1Y1X2Y2_PositiveOrder)
{
  auto c = Candidate::from_x1y1x2y2(10, 20, 50, 70, 0.8f);
  EXPECT_EQ(c.x1, 10);
  EXPECT_EQ(c.y1, 20);
  EXPECT_EQ(c.width, 40);
  EXPECT_EQ(c.height, 50);
  EXPECT_FLOAT_EQ(c.rating, 0.8f);
}

TEST(Candidate, FromX1Y1X2Y2_ReversedOrder)
{
  // x2 < x1 and y2 < y1 — should still produce a valid box
  auto c = Candidate::from_x1y1x2y2(50, 70, 10, 20, 0.5f);
  EXPECT_EQ(c.x1, 10);
  EXPECT_EQ(c.y1, 20);
  EXPECT_EQ(c.width, 40);
  EXPECT_EQ(c.height, 50);
}

TEST(Candidate, FromX1Y1X2Y2_Degenerate)
{
  // Point box
  auto c = Candidate::from_x1y1x2y2(5, 5, 5, 5, 1.0f);
  EXPECT_EQ(c.width, 0);
  EXPECT_EQ(c.height, 0);
}

// ---------------------------------------------------------------------------
// Derived accessors
// ---------------------------------------------------------------------------

TEST(Candidate, CenterXY)
{
  auto c = Candidate::from_x1y1x2y2(10, 20, 50, 60, 1.0f);
  // x1=10, width=40 → center_x = 10 + 20 = 30
  // y1=20, height=40 → center_y = 20 + 20 = 40
  EXPECT_EQ(c.center_x(), 30);
  EXPECT_EQ(c.center_y(), 40);
}

TEST(Candidate, X2Y2)
{
  auto c = Candidate::from_x1y1x2y2(10, 20, 50, 70, 1.0f);
  EXPECT_EQ(c.x2(), 50);
  EXPECT_EQ(c.y2(), 70);
}

TEST(Candidate, Radius_SquareBox)
{
  auto c = Candidate::from_x1y1x2y2(0, 0, 40, 40, 1.0f);
  // (width + height) / 4 = (40 + 40) / 4 = 20
  EXPECT_EQ(c.radius(), 20);
}

TEST(Candidate, Radius_RectBox)
{
  auto c = Candidate::from_x1y1x2y2(0, 0, 20, 60, 1.0f);
  // (20 + 60) / 4 = 20
  EXPECT_EQ(c.radius(), 20);
}

// ---------------------------------------------------------------------------
// sort_by_rating
// ---------------------------------------------------------------------------

TEST(Candidate, SortByRating_OrderedDescending)
{
  std::vector<Candidate> v = {
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.3f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.9f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.6f),
  };
  auto sorted = Candidate::sort_by_rating(v);
  ASSERT_EQ(sorted.size(), 3u);
  EXPECT_FLOAT_EQ(sorted[0].rating, 0.9f);
  EXPECT_FLOAT_EQ(sorted[1].rating, 0.6f);
  EXPECT_FLOAT_EQ(sorted[2].rating, 0.3f);
}

TEST(Candidate, SortByRating_DoesNotModifyOriginal)
{
  std::vector<Candidate> v = {
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.1f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.9f),
  };
  auto sorted = Candidate::sort_by_rating(v);
  // Original is unchanged (sort is by value)
  EXPECT_FLOAT_EQ(v[0].rating, 0.1f);
  EXPECT_FLOAT_EQ(v[1].rating, 0.9f);
  EXPECT_FLOAT_EQ(sorted[0].rating, 0.9f);
}

TEST(Candidate, SortByRating_Empty)
{
  std::vector<Candidate> v;
  auto sorted = Candidate::sort_by_rating(v);
  EXPECT_TRUE(sorted.empty());
}

// ---------------------------------------------------------------------------
// filter_by_rating
// ---------------------------------------------------------------------------

TEST(Candidate, FilterByRating_KeepsAboveThreshold)
{
  std::vector<Candidate> v = {
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.3f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.5f),  // exactly at threshold — excluded (strict >)
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.9f),
  };
  auto filtered = Candidate::filter_by_rating(v, 0.5f);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_FLOAT_EQ(filtered[0].rating, 0.9f);
}

TEST(Candidate, FilterByRating_AllBelowThreshold)
{
  std::vector<Candidate> v = {
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.1f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.2f),
  };
  auto filtered = Candidate::filter_by_rating(v, 0.5f);
  EXPECT_TRUE(filtered.empty());
}

TEST(Candidate, FilterByRating_Empty)
{
  std::vector<Candidate> v;
  auto filtered = Candidate::filter_by_rating(v, 0.5f);
  EXPECT_TRUE(filtered.empty());
}

TEST(Candidate, FilterByRating_AllAboveThreshold)
{
  std::vector<Candidate> v = {
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.8f),
    Candidate::from_x1y1x2y2(0, 0, 1, 1, 0.9f),
  };
  auto filtered = Candidate::filter_by_rating(v, 0.0f);
  EXPECT_EQ(filtered.size(), 2u);
}
