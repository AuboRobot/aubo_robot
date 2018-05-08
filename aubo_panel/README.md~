If you meet this error:
	usr/include/boost/type_traits/detail/has_binary_operator.hp:51: Parse error at "BOOST_JOIN"

You can solved this problem by:

located to "/usr/include/boost/type_traits/detail/has_binary_operator.hpp" 

In this file:


namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {

..

}

changed to

#ifndef Q_MOC_RUN
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
#endif

...

#ifndef Q_MOC_RUN
}
#endif
