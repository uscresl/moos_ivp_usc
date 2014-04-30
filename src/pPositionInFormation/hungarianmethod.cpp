#include "hungarianmethod.h"
#include <iostream>
#include <set>

HungarianMethod::HungarianMethod()
{
  // TODO Auto-generated constructor stub
}

HungarianMethod::~HungarianMethod()
{
  // TODO Auto-generated destructor stub
}

Eigen::VectorXi HungarianMethod::hungarian_solve(const Eigen::MatrixXd& c) {
  const double tolerance = 1e-6;

  int m = c.rows();
  int n = c.cols();
  Eigen::MatrixXd cost_matrix;
  int size = std::max(m, n);

  Eigen::VectorXi assignment;
  assignment.resize(c.rows());

  cost_matrix.resize(size, size);
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      if (i < m && j < n) {
        cost_matrix(i, j) = c(i, j);
      } else {
        cost_matrix(i, j) = 0;
      }
      if (std::isnan(cost_matrix(i, j))) {
        std::cerr << "dist is NaN: c=" << c(i, j) << std::endl;
        exit(-1);
      }
    }
  }
  m = n = size;

  int i, j, k, l, t, q, unmatched;

  double s, cost;

  cost = 0;
  Eigen::VectorXi col_mate;
  col_mate.resize(m);
  Eigen::VectorXi unchosen_row;
  unchosen_row.resize(m);
  Eigen::VectorXd row_dec;
  row_dec.resize(m);
  Eigen::VectorXi slack_row;
  slack_row.resize(m);

  Eigen::VectorXi row_mate;
  row_mate.resize(n);
  Eigen::VectorXi parent_row;
  parent_row.resize(n);
  Eigen::VectorXd col_inc;
  col_inc.resize(n);
  Eigen::VectorXd slack;
  slack.resize(n);

  for (i = 0; i < m; i++) {
    col_mate(i) = 0;
    unchosen_row(i) = 0;
    row_dec(i) = 0;
    slack_row(i) = 0;
  }
  for (j = 0; j < n; j++) {
    row_mate(j) = 0;
    parent_row(j) = 0;
    col_inc(j) = 0;
    slack(j) = 0;
  }

  for (l = 0; l < n; l++) {
    s = cost_matrix(0, l);
    for (k = 1; k < m; k++) {
      if (cost_matrix(k, l) < s) {
        s = cost_matrix(k, l);
      }
    }
    cost += s;
    if (s != 0)
      for (k = 0; k < m; k++)
        cost_matrix(k, l) -= s;
  }
// End subtract column minima in order to start with lots of zeroes 12

// Begin initial state 16
  t = 0;
  for (l = 0; l < n; l++)
      {
    row_mate(l) = -1;
    parent_row(l) = -1;
    col_inc(l) = 0;
    slack(l) = INF;
  }
  for (k = 0; k < m; k++)
      {
    s = cost_matrix(k, 0);
    for (l = 1; l < n; l++)
      if (cost_matrix(k, l) < s)
        s = cost_matrix(k, l);
    row_dec(k) = s;
    for (l = 0; l < n; l++) {
      if (std::abs(s - cost_matrix(k, l)) < tolerance && row_mate(l) < 0)
          {
        col_mate(k) = l;
        row_mate(l) = k;
        goto row_done;
      }
    }
    col_mate(k) = -1;
    unchosen_row(t++) = k;
    row_done:
    ;
  }
// End initial state 16

// Begin Hungarian algorithm 18
  if (t == 0)
    goto done;
  unmatched = t;
  while (1)
  {
    q = 0;
    while (1)
    {
      while (q < t) {
        // Begin explore node q of the forest 19
        {
          k = unchosen_row(q);
          s = row_dec(k);
          for (l = 0; l < n; l++) {
            if (slack(l)) {
              double del = cost_matrix(k, l) - s + col_inc(l);
              if (del < slack(l))
                  {
                if (std::abs(del) < tolerance)
                    {
                  if (row_mate(l) < 0) {
                    goto breakthru;
                  }
                  slack(l) = 0;
                  parent_row(l) = k;
                  unchosen_row(t++) = row_mate(l);
                }
                else
                {
                  slack(l) = del;
                  slack_row(l) = k;
                }
              }
            }
          }
        }
        // End explore node q of the forest 19
        q++;
      }

      // Begin introduce a new zero into the matrix 21
      s = INF;
      for (l = 0; l < n; l++) {
        if (slack(l) && slack(l) < s)
          s = slack(l);
      }
      for (q = 0; q < t; q++)
        row_dec(unchosen_row(q)) += s;
      for (l = 0; l < n; l++)
        if (slack(l))
            {
          slack(l) -= s;
          if (slack(l) == 0)
              {
            // Begin look at a new zero 22
            k = slack_row(l);
            if (row_mate(l) < 0)
                {
              for (j = l + 1; j < n; j++)
                if (slack(j) == 0)
                  col_inc(j) += s;
              goto breakthru;
            }
            else
            {
              parent_row(l) = k;
              unchosen_row(t++) = row_mate(l);
            }
            // End look at a new zero 22
          }
        }
        else
          col_inc(l) += s;
      // End introduce a new zero into the matrix 21
    }
    breakthru:
    // Begin update the matching 20
    while (1)
    {
      j = col_mate(k); //TODO: Here k is sometimes -1
      col_mate(k) = l;
      row_mate(l) = k;
      if (j < 0)
        break;
      k = parent_row(j);
      l = j;
    }
    // End update the matching 20
    if (--unmatched == 0)
      goto done;
    // Begin get ready for another stage 17
    t = 0;
    for (l = 0; l < n; l++)
        {
      parent_row(l) = -1;
      slack(l) = INF;
    }
    for (k = 0; k < m; k++)
      if (col_mate(k) < 0)
          {
        unchosen_row(t++) = k;
      }
    // End get ready for another stage 17
  }
  done:

// Begin doublecheck the solution 23
  for (k = 0; k < m; k++) {
    for (l = 0; l < n; l++) {
      if (cost_matrix(k, l) < ((row_dec(k) - col_inc(l)) - tolerance)) {
        std::cerr << "Solution check 1 failed" << std::endl;
        std::cerr << "Cost matrix:\n" << cost_matrix << std::endl;
        exit(-1);
      }
    }
  }
  for (k = 0; k < m; k++) {
    l = col_mate(k);
    if (l < 0 || std::abs(cost_matrix(k, l) - (row_dec(k) - col_inc(l))) > tolerance) {
      std::cerr << "Solution check 2 failed" << std::endl;
      exit(-1);
    }
  }
  k = 0;
  for (l = 0; l < n; l++) {
    if (col_inc(l)) {
      k++;
    }
  }
  if (k > m) {
    std::cerr << "Solution check 3 failed" << std::endl;
    exit(-1);
  }
// End doublecheck the solution 23
// End Hungarian algorithm 18

  for (i = 0; i < assignment.size(); ++i) {
    if (col_mate(i) < c.cols()) {
      assignment(i) = col_mate(i);
    } else {
      assignment(i) = -1;
    }
  }

  // Check if solution is valid 1 to 1 association
  std::set<int> testSet;
  cost = 0;
  double max_cost = 0;
  for (int i = 0; i < assignment.size(); ++i) {
    if (assignment(i) != -1) {
      testSet.insert(assignment(i));
      cost += c(i, assignment(i));
      max_cost = std::max(max_cost, c(i, assignment(i)));
    }
  }
  if ((int) testSet.size() != std::min(c.rows(), c.cols())) {
    std::cerr << "Hungarian method failed to find 1 to 1 association" << std::endl;
    exit(-1);
  }
  return assignment;
}

