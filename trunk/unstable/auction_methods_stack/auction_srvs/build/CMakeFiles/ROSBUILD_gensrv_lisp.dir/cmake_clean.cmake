FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/auction_srvs/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/BuyerService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_BuyerService.lisp"
  "../srv_gen/lisp/AuctioneerService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AuctioneerService.lisp"
  "../srv_gen/lisp/AuctioneerBidService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AuctioneerBidService.lisp"
  "../srv_gen/lisp/AuctionService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AuctionService.lisp"
  "../srv_gen/lisp/BuyerForwardBidService.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_BuyerForwardBidService.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
