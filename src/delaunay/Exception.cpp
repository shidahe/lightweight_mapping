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

#ifndef __DLOVI_EXCEPTION_CPP
#define __DLOVI_EXCEPTION_CPP

#include "Exception.h"
#include <iostream>
#include <sstream>

namespace dlovi {

  // Constructors and Destructors

  Exception::Exception(const std::string & strMessage)
    : exception(), m_strMessage(strMessage), m_bTagged(false) { if(m_strMessage.empty()) m_strMessage = "no message"; }

  Exception::~Exception() throw() { }

  // Public Methods:

  const char * Exception::what() const throw() {
    try {
      std::stringstream ssMessage;

      if (m_bTagged)
        ssMessage << "Exception in " << m_strMessage;
      else
        ssMessage << "Exception: " << m_strMessage;

      m_strFormattedMessage = ssMessage.str();
      return m_strFormattedMessage.c_str();
    }
    catch (std::exception & e) {
      std::cerr << "Warning in: dlovi::Exception::what(): Failed to format message. (msg = " << m_strMessage << ")" << std::endl;
      return "Exception: error formatting exception text";
    }
  }

  void Exception::raise() {
    throw *this;
  }

  void Exception::tag(const std::string & strClass, const std::string & strFunction) throw () {
    try {
      std::stringstream ssMessage;
      ssMessage << strClass << "::" << strFunction << "()" << (m_bTagged ? " -> " : ": ") << m_strMessage;
      m_strMessage = ssMessage.str();
      m_bTagged = true;
    }
    catch (std::exception & e) {
      std::cerr << "Warning in: dlovi::Exception::tag(): Failed to tag.  (class = " << strClass << ", func = " << strFunction << ")"  << std::endl;
    }
  }

  void Exception::tag(const std::string & strFunction) throw () {
    try {
      std::stringstream ssMessage;
      ssMessage << strFunction << "()" << (m_bTagged ? " -> " : ": ") << m_strMessage;
      m_strMessage = ssMessage.str();
      m_bTagged = true;
    }
    catch (std::exception & e) {
      std::cerr << "Warning in: dlovi::Exception::tag(): Failed to tag.  (func = " << strFunction << ")"  << std::endl;
    }
  }

}

#endif
