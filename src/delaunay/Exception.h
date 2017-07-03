//  Copyright 2011 David Lovi
//
//  This file is part of FreespaceDelaunayAlgorithm.
//
//  FreespaceDelaunayAlgorithm is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  FreespaceDelaunayAlgorithm is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with FreespaceDelaunayAlgorithm.  If not, see
//  <http://www.gnu.org/licenses/>.
//
//  As a special exception, you have permission to link this program
//  with the CGAL library and distribute executables, as long as you
//  follow the requirements of the GNU GPL in regard to all of the
//  software in the executable aside from CGAL.


// This header defines a custom exception class.  It has convenience routines
// For tagging where the exception was thrown from.

#ifndef __DLOVI_EXCEPTION_H
#define __DLOVI_EXCEPTION_H

#include <exception>
#include <string>

namespace dlovi {
  class Exception : public std::exception {
  public:
    // Constructors and Destructors
    Exception(const std::string & strMessage);
    virtual ~Exception() throw();

    // Public Methods
    virtual const char * what() const throw();
    virtual void raise();
    void tag(const std::string & strClass, const std::string & strFunction) throw ();
    void tag(const std::string & strFunction) throw ();

  protected:
    // Members
    std::string m_strMessage;
    mutable std::string m_strFormattedMessage;
    bool m_bTagged;
  };
}

#endif
